`default_nettype none

/*
============================================================================
bnn_soc_ctrl (AXI4-Lite 从设备控制寄存器与 MMIO 单条指令直推单元)
----------------------------------------------------------------------------
模块职责：
参考 TVM VTA 的 VCR (VTA Control Registers) AXI4-Lite 从设备结构，简化为纯 MMIO
单条 64-bit 指令直推模式：CPU 通过写 CMD_LO / CMD_HI 装填指令并写 CTRL_STATUS[0]
触发执行，轮询 DONE（或等待 irq 中断）完成单步闭环交互，无需 DDR 指令预取状态机。

寄存器映射表（32-bit 字寻址，4-Byte 对齐，按 addr[5:2] 译码）：
  偏移    名称            属性    复位值          功能说明
  0x00    CTRL_STATUS     RW      32'h0000_0000   控制与状态寄存器：
                                                  [0] LAUNCH     (RW)  ：空闲时写 1 向核心提交 {CMD_HI, CMD_LO}；
                                                                         指令完成或出错时硬件自动清 0
                                                  [1] DONE       (RW1C)：单条指令结束（cmd_done 或出错）时硬件置 1；
                                                                         写 1 清零，或下次有效 LAUNCH 时自动清零
                                                  [2] BUSY       (RO)  ：指令等待握手/执行中、core_busy 或 dma_busy
                                                  [3] ERROR      (RW1C)：指令报错标志（cmd_error 或 AXI 读写错误）；
                                                                         写 1 清零，或下次有效 LAUNCH 时自动清零
                                                  [4] SOFT_RESET (WO)  ：在 !dma_busy 时写 1 发出 1 拍 soft_reset 脉冲，
                                                                         同步复位 bnn_core_top 与计数状态
                                                  [8] IRQ_EN     (RW)  ：中断使能，irq = DONE & IRQ_EN
  0x04    CYCLE_CNT       RW      32'h0000_0000   执行周期累计计数器（BUSY 期间每拍饱和 +1，软件可写 0 清零）
  0x08    CMD_LO          RW      32'h0000_0000   待直推 64-bit 指令低 32 位 [31:0] ({flags, N, M, opcode})
  0x0C    CMD_HI          RW      32'h0000_0000   待直推 64-bit 指令高 32 位 [63:32] (addr / row_stride)
  0x10    DATA_BASE_LO    RW      32'h0000_0000   数据访存物理基址偏移低 32 位（默认 0 表示 CMD_HI 即物理地址；
                                                  非 0 时 axi_addr = DATA_BASE + core_addr）
  0x14    DATA_BASE_HI    RW      32'h0000_0000   数据访存物理基址偏移高 32 位
  0x18    CMD_DONE_CNT    RW      32'h0000_0000   累计成功完成指令数（每次无错完成自动 +1，软件可写 0 清零）
  0x1C    ERR_CODE        RO      32'h0000_0000   错误原因位图（随 ERROR 标志同步清零）：
                                                  [0] CORE_CMD_ERR    ：bnn_core_top 返回 cmd_error
                                                  [1] BUSY_LAUNCH_ERR ：BUSY 期间重复写 LAUNCH=1
                                                  [2] AXI_RD_ERR      ：m_axi_gmem RRESP != 2'b00 (OKAY)
                                                  [3] AXI_WR_ERR      ：m_axi_gmem BRESP != 2'b00 (OKAY)
  0x20    CORE_ID         RO      32'h424E_4E32   固定魔数标识 "BNN2"（供软件验证 AXI4-Lite 映射）

典型软件单条指令提交流程：
  1. 写 CMD_LO (0x08) 与 CMD_HI (0x0C)
  2. 写 CTRL_STATUS (0x00) = 32'h1 (LAUNCH)
  3. 轮询 CTRL_STATUS 直至 bit[1] (DONE) == 1，检查 bit[3] (ERROR) == 0
============================================================================
*/
module bnn_soc_ctrl #(
    parameter int unsigned C_S_AXI_ADDR_WIDTH = 16,
    parameter int unsigned C_S_AXI_DATA_WIDTH = 32
) (
    input  wire                                  clock,
    input  wire                                  ext_reset,
    output wire                                  soft_reset,
    output wire                                  irq,

    // ------------------------------------------------------------------------
    // AXI4-Lite Slave 接口 (s_axi_control_*)
    // ------------------------------------------------------------------------
    input  wire                                  s_axi_control_AWVALID,
    output wire                                  s_axi_control_AWREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]         s_axi_control_AWADDR,

    input  wire                                  s_axi_control_WVALID,
    output wire                                  s_axi_control_WREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]         s_axi_control_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]     s_axi_control_WSTRB,

    output wire                                  s_axi_control_BVALID,
    input  wire                                  s_axi_control_BREADY,
    output wire [1:0]                            s_axi_control_BRESP,

    input  wire                                  s_axi_control_ARVALID,
    output wire                                  s_axi_control_ARREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]         s_axi_control_ARADDR,

    output wire                                  s_axi_control_RVALID,
    input  wire                                  s_axi_control_RREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0]         s_axi_control_RDATA,
    output wire [1:0]                            s_axi_control_RRESP,

    // ------------------------------------------------------------------------
    // 去往 bnn_core_top 的单阶段 64-bit 原子指令与状态接口
    // ------------------------------------------------------------------------
    output wire                                  cmd_valid,
    output wire [63:0]                           cmd_inst,
    input  wire                                  cmd_ready,
    input  wire                                  cmd_done,
    input  wire                                  cmd_error,
    input  wire                                  core_busy,

    // ------------------------------------------------------------------------
    // 去往 bnn_soc_dma 的基址配置与状态观测接口
    // ------------------------------------------------------------------------
    output wire [63:0]                           data_base_addr,
    input  wire                                  dma_busy,
    input  wire                                  axi_rd_err,
    input  wire                                  axi_wr_err
);
    localparam logic [31:0] CORE_MAGIC_ID = 32'h424E_4E32; // "BNN2"

    typedef enum logic [1:0] {
        CMD_IDLE,
        CMD_ISSUE,
        CMD_WAIT,
        CMD_DRAIN
    } cmd_state_t;

    cmd_state_t  cmd_state;
    logic [63:0] cmd_latch;
    logic        soft_reset_reg;
    logic        done_flag;
    logic        error_flag;
    logic        irq_en_reg;
    logic        run_had_err;
    logic [3:0]  err_code_reg;

    logic [31:0] cycle_cnt_reg;
    logic [31:0] cmd_lo_reg;
    logic [31:0] cmd_hi_reg;
    logic [31:0] data_base_lo_reg;
    logic [31:0] data_base_hi_reg;
    logic [31:0] cmd_done_cnt_reg;

    // ------------------------------------------------------------------------
    // AXI4-Lite 写通道解耦捕获（支持 AW 与 W 同拍或任意先后到达，无组合环）
    // ------------------------------------------------------------------------
    logic                            aw_captured;
    logic [C_S_AXI_ADDR_WIDTH-1:0]   aw_addr_reg;
    logic                            w_captured;
    logic [31:0]                     w_data_reg;
    logic [3:0]                      w_strb_reg;
    logic                            b_valid_reg;

    wire aw_fire = s_axi_control_AWVALID && s_axi_control_AWREADY;
    wire w_fire  = s_axi_control_WVALID  && s_axi_control_WREADY;
    wire wr_commit = (aw_captured || aw_fire) && (w_captured || w_fire);

    wire [C_S_AXI_ADDR_WIDTH-1:0] wr_addr_eff = aw_captured ? aw_addr_reg : s_axi_control_AWADDR;
    wire [31:0]                   wr_data_eff = w_captured  ? w_data_reg  : s_axi_control_WDATA;
    wire [3:0]                    wr_strb_eff = w_captured  ? w_strb_reg  : s_axi_control_WSTRB;
    wire [3:0]                    wr_word_idx = wr_addr_eff[5:2];
    // 仅低 64B 映射区（addr[C_S_AXI_ADDR_WIDTH-1:6] == 0）命中有效寄存器
    wire                          wr_in_range = (wr_addr_eff[C_S_AXI_ADDR_WIDTH-1:6] == '0);
    wire                          wr_reg_hit  = wr_commit && wr_in_range;

    assign s_axi_control_AWREADY = !ext_reset && !aw_captured && !b_valid_reg;
    assign s_axi_control_WREADY  = !ext_reset && !w_captured  && !b_valid_reg;
    assign s_axi_control_BVALID  = !ext_reset && b_valid_reg;
    assign s_axi_control_BRESP   = 2'b00;

    // ------------------------------------------------------------------------
    // AXI4-Lite 读通道
    // ------------------------------------------------------------------------
    logic        r_valid_reg;
    logic [31:0] r_data_reg;

    wire ar_fire     = s_axi_control_ARVALID && s_axi_control_ARREADY;
    wire rd_in_range = (s_axi_control_ARADDR[C_S_AXI_ADDR_WIDTH-1:6] == '0);
    wire [3:0] rd_word_idx = s_axi_control_ARADDR[5:2];

    assign s_axi_control_ARREADY = !ext_reset && !r_valid_reg;
    assign s_axi_control_RVALID  = !ext_reset && r_valid_reg;
    assign s_axi_control_RDATA   = ext_reset ? 32'd0 : r_data_reg;
    assign s_axi_control_RRESP   = 2'b00;

    // ------------------------------------------------------------------------
    // 状态与对外控制信号导出
    // ------------------------------------------------------------------------
    wire launch_active = (cmd_state != CMD_IDLE);
    wire busy          = launch_active || core_busy || dma_busy;

    wire [31:0] ctrl_status_rdata = {
        23'd0,
        irq_en_reg,    // [8] IRQ_EN
        3'd0,          // [7:5] reserved
        1'b0,          // [4] SOFT_RESET (WO)
        error_flag,    // [3] ERROR
        busy,          // [2] BUSY
        done_flag,     // [1] DONE
        launch_active  // [0] LAUNCH
    };

    assign soft_reset     = !ext_reset && soft_reset_reg;
    assign irq            = !ext_reset && (done_flag && irq_en_reg);
    assign cmd_valid      = !ext_reset && !soft_reset_reg && (cmd_state == CMD_ISSUE);
    assign cmd_inst       = cmd_latch;
    assign data_base_addr = {data_base_hi_reg, data_base_lo_reg};

    function automatic logic [31:0] apply_wstrb(
        input logic [31:0] old_val,
        input logic [31:0] new_val,
        input logic [3:0]  strb
    );
        logic [31:0] merged;
        for (int b = 0; b < 4; b++) begin
            merged[b*8 +: 8] = strb[b] ? new_val[b*8 +: 8] : old_val[b*8 +: 8];
        end
        return merged;
    endfunction

    logic [31:0] rd_mux_data;
    always_comb begin
        rd_mux_data = 32'd0;
        if (rd_in_range) begin
            case (rd_word_idx)
                4'h0: rd_mux_data = ctrl_status_rdata;
                4'h1: rd_mux_data = cycle_cnt_reg;
                4'h2: rd_mux_data = cmd_lo_reg;
                4'h3: rd_mux_data = cmd_hi_reg;
                4'h4: rd_mux_data = data_base_lo_reg;
                4'h5: rd_mux_data = data_base_hi_reg;
                4'h6: rd_mux_data = cmd_done_cnt_reg;
                4'h7: rd_mux_data = {28'd0, err_code_reg};
                4'h8: rd_mux_data = CORE_MAGIC_ID;
                default: rd_mux_data = 32'd0;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // AXI4-Lite 通道握手时序（仅受 ext_reset 复位，保证 SOFT_RESET 不丢失 BVALID）
    // ------------------------------------------------------------------------
    always_ff @(posedge clock) begin
        if (ext_reset) begin
            aw_captured <= 1'b0;
            aw_addr_reg <= '0;
            w_captured  <= 1'b0;
            w_data_reg  <= 32'd0;
            w_strb_reg  <= 4'd0;
            b_valid_reg <= 1'b0;
            r_valid_reg <= 1'b0;
            r_data_reg  <= 32'd0;
        end else begin
            if (wr_commit) begin
                aw_captured <= 1'b0;
                w_captured  <= 1'b0;
                b_valid_reg <= 1'b1;
            end else begin
                if (aw_fire) begin
                    aw_captured <= 1'b1;
                    aw_addr_reg <= s_axi_control_AWADDR;
                end
                if (w_fire) begin
                    w_captured <= 1'b1;
                    w_data_reg <= s_axi_control_WDATA;
                    w_strb_reg <= s_axi_control_WSTRB;
                end
                if (b_valid_reg && s_axi_control_BREADY) begin
                    b_valid_reg <= 1'b0;
                end
            end

            if (ar_fire) begin
                r_valid_reg <= 1'b1;
                r_data_reg  <= rd_mux_data;
            end else if (r_valid_reg && s_axi_control_RREADY) begin
                r_valid_reg <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 控制寄存器、性能计数器与单条指令提交状态机
    // ------------------------------------------------------------------------
    wire do_soft_reset = wr_reg_hit && (wr_word_idx == 4'h0)
                      && wr_strb_eff[0] && wr_data_eff[4] && !dma_busy;

    wire any_err_now = run_had_err || axi_rd_err || axi_wr_err;

    always_ff @(posedge clock) begin
        if (ext_reset) begin
            cmd_state        <= CMD_IDLE;
            cmd_latch        <= 64'd0;
            soft_reset_reg   <= 1'b0;
            done_flag        <= 1'b0;
            error_flag       <= 1'b0;
            irq_en_reg       <= 1'b0;
            run_had_err      <= 1'b0;
            err_code_reg     <= 4'd0;
            cycle_cnt_reg    <= 32'd0;
            cmd_lo_reg       <= 32'd0;
            cmd_hi_reg       <= 32'd0;
            data_base_lo_reg <= 32'd0;
            data_base_hi_reg <= 32'd0;
            cmd_done_cnt_reg <= 32'd0;
        end else begin
            soft_reset_reg <= 1'b0;

            if (do_soft_reset) begin
                soft_reset_reg   <= 1'b1;
                cmd_state        <= CMD_IDLE;
                done_flag        <= 1'b0;
                error_flag       <= 1'b0;
                run_had_err      <= 1'b0;
                err_code_reg     <= 4'd0;
                cycle_cnt_reg    <= 32'd0;
                cmd_done_cnt_reg <= 32'd0;
                if (wr_strb_eff[1]) begin
                    irq_en_reg <= wr_data_eff[8];
                end
            end else begin
                // 1. 持续捕获取自 DMA 的 AXI 响应错误脉冲
                if (axi_rd_err) begin
                    error_flag      <= 1'b1;
                    err_code_reg[2] <= 1'b1;
                    run_had_err     <= 1'b1;
                end
                if (axi_wr_err) begin
                    error_flag      <= 1'b1;
                    err_code_reg[3] <= 1'b1;
                    run_had_err     <= 1'b1;
                end

                // 2. 周期计数器（写优先，否则 BUSY 期间每拍饱和 +1）
                if (wr_reg_hit && (wr_word_idx == 4'h1)) begin
                    cycle_cnt_reg <= apply_wstrb(cycle_cnt_reg, wr_data_eff, wr_strb_eff);
                end else if (busy && (cycle_cnt_reg != 32'hFFFF_FFFF)) begin
                    cycle_cnt_reg <= cycle_cnt_reg + 32'd1;
                end

                // 3. 普通配置寄存器写入
                if (wr_reg_hit) begin
                    case (wr_word_idx)
                        4'h0: begin
                            if (wr_strb_eff[1]) begin
                                irq_en_reg <= wr_data_eff[8];
                            end
                            if (wr_strb_eff[0]) begin
                                if (wr_data_eff[0]) begin
                                    // LAUNCH 触发单条指令执行
                                    if (!busy) begin
                                        cmd_latch    <= {cmd_hi_reg, cmd_lo_reg};
                                        cmd_state    <= CMD_ISSUE;
                                        done_flag    <= 1'b0;
                                        error_flag   <= 1'b0;
                                        err_code_reg <= 4'd0;
                                        run_had_err  <= 1'b0;
                                    end else begin
                                        error_flag      <= 1'b1;
                                        err_code_reg[1] <= 1'b1;
                                    end
                                end else begin
                                    if (wr_data_eff[1]) begin
                                        done_flag <= 1'b0;
                                    end
                                    if (wr_data_eff[3]) begin
                                        error_flag   <= 1'b0;
                                        err_code_reg <= 4'd0;
                                    end
                                end
                            end
                        end
                        4'h2: cmd_lo_reg       <= apply_wstrb(cmd_lo_reg,       wr_data_eff, wr_strb_eff);
                        4'h3: cmd_hi_reg       <= apply_wstrb(cmd_hi_reg,       wr_data_eff, wr_strb_eff);
                        4'h4: data_base_lo_reg <= apply_wstrb(data_base_lo_reg, wr_data_eff, wr_strb_eff);
                        4'h5: data_base_hi_reg <= apply_wstrb(data_base_hi_reg, wr_data_eff, wr_strb_eff);
                        4'h6: cmd_done_cnt_reg <= apply_wstrb(cmd_done_cnt_reg, wr_data_eff, wr_strb_eff);
                        default: ;
                    endcase
                end

                // 4. 指令派发与完成状态机
                case (cmd_state)
                    CMD_IDLE: ;
                    CMD_ISSUE: begin
                        if (cmd_ready) begin
                            cmd_state <= CMD_WAIT;
                        end
                    end
                    CMD_WAIT: begin
                        if (cmd_error) begin
                            error_flag      <= 1'b1;
                            err_code_reg[0] <= 1'b1;
                            run_had_err     <= 1'b1;
                            if (!core_busy && !dma_busy) begin
                                done_flag <= 1'b1;
                                cmd_state <= CMD_IDLE;
                            end else begin
                                cmd_state <= CMD_DRAIN;
                            end
                        end else if (cmd_done) begin
                            if (!core_busy && !dma_busy) begin
                                done_flag <= 1'b1;
                                if (any_err_now) begin
                                    error_flag <= 1'b1;
                                end else if (!(wr_reg_hit && (wr_word_idx == 4'h6)) &&
                                             (cmd_done_cnt_reg != 32'hFFFF_FFFF)) begin
                                    cmd_done_cnt_reg <= cmd_done_cnt_reg + 32'd1;
                                end
                                cmd_state <= CMD_IDLE;
                            end else begin
                                cmd_state <= CMD_DRAIN;
                            end
                        end
                    end
                    CMD_DRAIN: begin
                        if (!core_busy && !dma_busy) begin
                            done_flag <= 1'b1;
                            if (any_err_now) begin
                                error_flag <= 1'b1;
                            end else if (!(wr_reg_hit && (wr_word_idx == 4'h6)) &&
                                         (cmd_done_cnt_reg != 32'hFFFF_FFFF)) begin
                                cmd_done_cnt_reg <= cmd_done_cnt_reg + 32'd1;
                            end
                            cmd_state <= CMD_IDLE;
                        end
                    end
                    default: cmd_state <= CMD_IDLE;
                endcase
            end
        end
    end

    wire _unused_addr_lsbs = &{1'b0, wr_addr_eff[1:0], s_axi_control_ARADDR[1:0]};

`ifndef SYNTHESIS
    initial begin
        assert (C_S_AXI_ADDR_WIDTH >= 6)
            else $fatal(1, "C_S_AXI_ADDR_WIDTH must be >= 6 (at least 64B register space)");
        assert (C_S_AXI_DATA_WIDTH == 32)
            else $fatal(1, "C_S_AXI_DATA_WIDTH must be 32");
    end
`endif

endmodule

`default_nettype wire
