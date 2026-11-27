`default_nettype none

/*
============================================================================
bnn_soc_dma (BNN v2 核心 DMA <-> AXI4 Master 突发切分与位宽适配引擎)
----------------------------------------------------------------------------
模块职责：
参考 TVM VTA 的 VMESimple 访存桥接层，针对单条 MMIO 指令直推模式做极简专化（无取指读仲裁，
读/写通道各仅服务 bnn_core_top 唯一客户端）：
1. 地址重定位：将核心 32-bit 字节地址加上 64-bit data_base_addr 得到最终 AXI 物理地址。
2. 位宽齿轮箱适配（核心固定 DMA_DATA_WIDTH = 128）：
   - C_M_AXI_DATA_WIDTH == 64 （默认，适配 Zynq-7000 64-bit S_AXI_ACP / S_AXI_HP）：
     读方向每收齐 2 拍小端 64-bit RDATA 拼成 1 拍 128-bit 返回核心；
     写方向将核心每 1 拍 128-bit 数据与 16-bit strb 按小端拆成 2 拍 64-bit WDATA/WSTRB 发送。
   - C_M_AXI_DATA_WIDTH == 128（可选，适配 Zynq UltraScale+ 128-bit HP/HPC）：1:1 直通。
3. 突发合规切分：
   - 核心单次请求长度 mem_*_cmd_len（1..65535 拍 128-bit）换算为总 AXI 拍数后，自动按
     AXI_MAX_BURST_LEN（<= 256 拍）及 AXI4 4KiB (0x1000) 地址边界拆分为多笔顺序 INCR 突发。
   - 读通道仅在整笔核心请求的最后一拍 128-bit 数据上拉高 mem_rd_data_last。
   - 写通道在整笔核心请求拆出的最后一笔 AXI 写突发收到 BVALID 后，向核心发出 1 拍 mem_wr_done。
4. 总线错误上报：
   - 监测 RRESP != 2'b00 或 BRESP != 2'b00 时分别发出单拍 axi_rd_err / axi_wr_err 脉冲，
     同时正常走完剩余拍握手以避免核心状态机死锁。
============================================================================
*/
module bnn_soc_dma #(
    parameter int unsigned CORE_ADDR_WIDTH      = 32,
    parameter int unsigned DMA_DATA_WIDTH       = 128,
    parameter int unsigned DMA_LEN_WIDTH        = 16,

    parameter int unsigned C_M_AXI_ID_WIDTH     = 8,
    parameter int unsigned C_M_AXI_ADDR_WIDTH   = 32,
    parameter int unsigned C_M_AXI_DATA_WIDTH   = 64,
    parameter int unsigned C_M_AXI_AWUSER_WIDTH = 1,
    parameter int unsigned C_M_AXI_ARUSER_WIDTH = 1,
    parameter int unsigned C_M_AXI_WUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_RUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_BUSER_WIDTH  = 1,

    parameter bit          AXI_COHERENT         = 1'b1,
    parameter int unsigned AXI_MAX_BURST_LEN    = 256
) (
    input  wire                                  clock,
    input  wire                                  reset,

    // ------------------------------------------------------------------------
    // 基址配置与状态信号（对接 bnn_soc_ctrl）
    // ------------------------------------------------------------------------
    input  wire [63:0]                           data_base_addr,
    output wire                                  dma_busy,
    output wire                                  axi_rd_err,
    output wire                                  axi_wr_err,

    // ------------------------------------------------------------------------
    // 来自 bnn_core_top 的统一 DMA 读接口
    // ------------------------------------------------------------------------
    input  wire                                  mem_rd_cmd_valid,
    output wire                                  mem_rd_cmd_ready,
    input  wire [CORE_ADDR_WIDTH-1:0]            mem_rd_cmd_addr,
    input  wire [DMA_LEN_WIDTH-1:0]              mem_rd_cmd_len,

    output wire                                  mem_rd_data_valid,
    input  wire                                  mem_rd_data_ready,
    output wire [DMA_DATA_WIDTH-1:0]             mem_rd_data,
    output wire                                  mem_rd_data_last,

    // ------------------------------------------------------------------------
    // 来自 bnn_core_top 的统一 DMA 写接口
    // ------------------------------------------------------------------------
    input  wire                                  mem_wr_cmd_valid,
    output wire                                  mem_wr_cmd_ready,
    input  wire [CORE_ADDR_WIDTH-1:0]            mem_wr_cmd_addr,
    input  wire [DMA_LEN_WIDTH-1:0]              mem_wr_cmd_len,

    input  wire                                  mem_wr_data_valid,
    output wire                                  mem_wr_data_ready,
    input  wire [DMA_DATA_WIDTH-1:0]             mem_wr_data,
    input  wire [(DMA_DATA_WIDTH/8)-1:0]         mem_wr_data_strb,
    input  wire                                  mem_wr_data_last,

    output wire                                  mem_wr_done,

    // ------------------------------------------------------------------------
    // 对外 AXI4 Full Master 接口 (m_axi_gmem_*)
    // ------------------------------------------------------------------------
    // Write Address Channel (AW)
    output wire                                  m_axi_gmem_AWVALID,
    input  wire                                  m_axi_gmem_AWREADY,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]         m_axi_gmem_AWADDR,
    output wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_AWID,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0]       m_axi_gmem_AWUSER,
    output wire [7:0]                            m_axi_gmem_AWLEN,
    output wire [2:0]                            m_axi_gmem_AWSIZE,
    output wire [1:0]                            m_axi_gmem_AWBURST,
    output wire [1:0]                            m_axi_gmem_AWLOCK,
    output wire [3:0]                            m_axi_gmem_AWCACHE,
    output wire [2:0]                            m_axi_gmem_AWPROT,
    output wire [3:0]                            m_axi_gmem_AWQOS,
    output wire [3:0]                            m_axi_gmem_AWREGION,

    // Write Data Channel (W)
    output wire                                  m_axi_gmem_WVALID,
    input  wire                                  m_axi_gmem_WREADY,
    output wire [C_M_AXI_DATA_WIDTH-1:0]         m_axi_gmem_WDATA,
    output wire [(C_M_AXI_DATA_WIDTH/8)-1:0]     m_axi_gmem_WSTRB,
    output wire                                  m_axi_gmem_WLAST,
    output wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_WID,
    output wire [C_M_AXI_WUSER_WIDTH-1:0]        m_axi_gmem_WUSER,

    // Write Response Channel (B)
    input  wire                                  m_axi_gmem_BVALID,
    output wire                                  m_axi_gmem_BREADY,
    input  wire [1:0]                            m_axi_gmem_BRESP,
    input  wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_BID,
    input  wire [C_M_AXI_BUSER_WIDTH-1:0]        m_axi_gmem_BUSER,

    // Read Address Channel (AR)
    output wire                                  m_axi_gmem_ARVALID,
    input  wire                                  m_axi_gmem_ARREADY,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]         m_axi_gmem_ARADDR,
    output wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_ARID,
    output wire [C_M_AXI_ARUSER_WIDTH-1:0]       m_axi_gmem_ARUSER,
    output wire [7:0]                            m_axi_gmem_ARLEN,
    output wire [2:0]                            m_axi_gmem_ARSIZE,
    output wire [1:0]                            m_axi_gmem_ARBURST,
    output wire [1:0]                            m_axi_gmem_ARLOCK,
    output wire [3:0]                            m_axi_gmem_ARCACHE,
    output wire [2:0]                            m_axi_gmem_ARPROT,
    output wire [3:0]                            m_axi_gmem_ARQOS,
    output wire [3:0]                            m_axi_gmem_ARREGION,

    // Read Data Channel (R)
    input  wire                                  m_axi_gmem_RVALID,
    output wire                                  m_axi_gmem_RREADY,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]         m_axi_gmem_RDATA,
    input  wire [1:0]                            m_axi_gmem_RRESP,
    input  wire                                  m_axi_gmem_RLAST,
    input  wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_RID,
    input  wire [C_M_AXI_RUSER_WIDTH-1:0]        m_axi_gmem_RUSER
);
    // ------------------------------------------------------------------------
    // AXI4 静态总线属性常量与位宽换算参数
    // ------------------------------------------------------------------------
    localparam int unsigned AXI_BEAT_BYTES  = C_M_AXI_DATA_WIDTH / 8;
    localparam int unsigned AXI_BEAT_SHIFT  = $clog2(AXI_BEAT_BYTES);
    localparam int unsigned SUBBEATS        = DMA_DATA_WIDTH / C_M_AXI_DATA_WIDTH;

    localparam logic [2:0] AXI_SIZE_CONST   = 3'(AXI_BEAT_SHIFT);
    localparam logic [1:0] AXI_BURST_INCR   = 2'b01;
    localparam logic [1:0] AXI_LOCK_NORMAL  = 2'b00;
    localparam logic [3:0] AXI_CACHE_CONST  = AXI_COHERENT ? 4'b1111 : 4'b0011;
    localparam logic [2:0] AXI_PROT_CONST   = AXI_COHERENT ? 3'b100  : 3'b000;
    localparam logic [3:0] AXI_QOS_CONST    = 4'b0000;
    localparam logic [3:0] AXI_REGION_CONST = 4'b0000;

    // ------------------------------------------------------------------------
    // 单笔突发拍数计算：同时满足 <= rem_beats、<= AXI_MAX_BURST_LEN 且不跨 4KiB 边界
    // ------------------------------------------------------------------------
    function automatic logic [8:0] calc_burst_beats(
        input logic [11:0] page_offset,
        input logic [31:0] rem_beats
    );
        logic [12:0] bytes_to_4k;
        logic [9:0]  raw_beats_to_4k;
        logic [9:0]  beats_to_4k;
        logic [8:0]  limit_max;
        bytes_to_4k     = 13'd4096 - {1'b0, page_offset};
        raw_beats_to_4k = 10'(bytes_to_4k >> AXI_BEAT_SHIFT);
        beats_to_4k     = (raw_beats_to_4k == 10'd0) ? 10'd1 : raw_beats_to_4k;
        limit_max       = (rem_beats < 32'(AXI_MAX_BURST_LEN)) ? 9'(rem_beats) : 9'(AXI_MAX_BURST_LEN);
        return ({1'b0, limit_max} < beats_to_4k) ? limit_max : 9'(beats_to_4k);
    endfunction

    // ========================================================================
    // 1. 读通道状态机与 64b/128b -> 128b 拼拍缓冲
    // ========================================================================
    typedef enum logic [2:0] {
        RD_IDLE,
        RD_CALC,
        RD_AR,
        RD_DATA,
        RD_DRAIN
    } rd_state_t;

    rd_state_t                    rd_state;
    logic [63:0]                  rd_cur_addr;
    logic [31:0]                  rd_plan_rem_beats;
    logic [C_M_AXI_ADDR_WIDTH-1:0] rd_araddr;
    logic [7:0]                   rd_arlen;
    logic [8:0]                   rd_burst_left;
    logic [31:0]                  rd_rx_total_left;

    logic                         rd_hold_valid;
    logic [DMA_DATA_WIDTH-1:0]    rd_hold_data;
    logic                         rd_hold_last;
    logic                         axi_rd_err_reg;

    wire                          rd_hold_can_load = !rd_hold_valid || mem_rd_data_ready;
    wire                          core_rd_consume  = rd_hold_valid && mem_rd_data_ready;
    wire                          r_fire           = m_axi_gmem_RVALID && m_axi_gmem_RREADY;

    wire                          rd_load_hold;
    wire [DMA_DATA_WIDTH-1:0]     rd_assembled_data;
    wire                          rd_assembled_last = (rd_rx_total_left == 32'd1);

    generate
        if (C_M_AXI_DATA_WIDTH == 64) begin : g_rd_64
            logic        rd_sub_idx;
            logic [63:0] rd_lo_buf;

            assign m_axi_gmem_RREADY = !reset && (rd_state == RD_DATA)
                                    && ((rd_sub_idx == 1'b0) || rd_hold_can_load);
            assign rd_load_hold      = r_fire && (rd_sub_idx == 1'b1);
            assign rd_assembled_data = {m_axi_gmem_RDATA, rd_lo_buf};

            always_ff @(posedge clock) begin
                if (reset) begin
                    rd_sub_idx <= 1'b0;
                    rd_lo_buf  <= 64'd0;
                end else if (rd_state == RD_IDLE) begin
                    rd_sub_idx <= 1'b0;
                end else if (r_fire) begin
                    if (rd_sub_idx == 1'b0) begin
                        rd_lo_buf  <= m_axi_gmem_RDATA;
                        rd_sub_idx <= 1'b1;
                    end else begin
                        rd_sub_idx <= 1'b0;
                    end
                end
            end
        end else begin : g_rd_128
            assign m_axi_gmem_RREADY = !reset && (rd_state == RD_DATA) && rd_hold_can_load;
            assign rd_load_hold      = r_fire;
            assign rd_assembled_data = m_axi_gmem_RDATA;
        end
    endgenerate

    wire [8:0] rd_next_burst = calc_burst_beats(rd_cur_addr[11:0], rd_plan_rem_beats);

    always_ff @(posedge clock) begin
        if (reset) begin
            rd_state          <= RD_IDLE;
            rd_cur_addr       <= 64'd0;
            rd_plan_rem_beats <= 32'd0;
            rd_araddr         <= '0;
            rd_arlen          <= 8'd0;
            rd_burst_left     <= 9'd0;
            rd_rx_total_left  <= 32'd0;
            rd_hold_valid     <= 1'b0;
            rd_hold_data      <= '0;
            rd_hold_last      <= 1'b0;
            axi_rd_err_reg    <= 1'b0;
        end else begin
            axi_rd_err_reg <= 1'b0;

            if (r_fire && (m_axi_gmem_RRESP != 2'b00)) begin
                axi_rd_err_reg <= 1'b1;
            end

            // 128-bit 输出保持寄存器更新（支持同拍消费与装填新拍无缝背靠背）
            if (rd_load_hold) begin
                rd_hold_valid <= 1'b1;
                rd_hold_data  <= rd_assembled_data;
                rd_hold_last  <= rd_assembled_last;
            end else if (core_rd_consume) begin
                rd_hold_valid <= 1'b0;
                rd_hold_last  <= 1'b0;
            end

            case (rd_state)
                RD_IDLE: begin
                    if (mem_rd_cmd_valid && mem_rd_cmd_ready) begin
                        rd_cur_addr       <= data_base_addr + 64'(mem_rd_cmd_addr);
                        rd_plan_rem_beats <= 32'(mem_rd_cmd_len) * 32'(SUBBEATS);
                        rd_rx_total_left  <= 32'(mem_rd_cmd_len) * 32'(SUBBEATS);
                        rd_state          <= RD_CALC;
                    end
                end
                RD_CALC: begin
                    rd_araddr         <= C_M_AXI_ADDR_WIDTH'(rd_cur_addr);
                    rd_arlen          <= 8'(rd_next_burst - 9'd1);
                    rd_burst_left     <= rd_next_burst;
                    rd_cur_addr       <= rd_cur_addr + (64'(rd_next_burst) << AXI_BEAT_SHIFT);
                    rd_plan_rem_beats <= rd_plan_rem_beats - 32'(rd_next_burst);
                    rd_state          <= RD_AR;
                end
                RD_AR: begin
                    if (m_axi_gmem_ARREADY) begin
                        rd_state <= RD_DATA;
                    end
                end
                RD_DATA: begin
                    if (r_fire) begin
                        rd_rx_total_left <= rd_rx_total_left - 32'd1;
                        if (rd_burst_left == 9'd1) begin
                            if (rd_plan_rem_beats == 32'd0) begin
                                rd_state <= RD_DRAIN;
                            end else begin
                                rd_state <= RD_CALC;
                            end
                        end else begin
                            rd_burst_left <= rd_burst_left - 9'd1;
                        end
                    end
                end
                RD_DRAIN: begin
                    if (!rd_hold_valid || core_rd_consume) begin
                        rd_state <= RD_IDLE;
                    end
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    assign mem_rd_cmd_ready    = !reset && (rd_state == RD_IDLE) && !rd_hold_valid;
    assign mem_rd_data_valid   = !reset && rd_hold_valid;
    assign mem_rd_data         = reset ? '0 : rd_hold_data;
    assign mem_rd_data_last    = !reset && rd_hold_valid && rd_hold_last;

    assign m_axi_gmem_ARVALID  = !reset && (rd_state == RD_AR);
    assign m_axi_gmem_ARADDR   = rd_araddr;
    assign m_axi_gmem_ARID     = '0;
    assign m_axi_gmem_ARUSER   = C_M_AXI_ARUSER_WIDTH'(AXI_COHERENT);
    assign m_axi_gmem_ARLEN    = rd_arlen;
    assign m_axi_gmem_ARSIZE   = AXI_SIZE_CONST;
    assign m_axi_gmem_ARBURST  = AXI_BURST_INCR;
    assign m_axi_gmem_ARLOCK   = AXI_LOCK_NORMAL;
    assign m_axi_gmem_ARCACHE  = AXI_CACHE_CONST;
    assign m_axi_gmem_ARPROT   = AXI_PROT_CONST;
    assign m_axi_gmem_ARQOS    = AXI_QOS_CONST;
    assign m_axi_gmem_ARREGION = AXI_REGION_CONST;

    // ========================================================================
    // 2. 写通道状态机与 128b -> 64b/128b 拆拍缓冲
    // ========================================================================
    typedef enum logic [2:0] {
        WR_IDLE,
        WR_CALC,
        WR_AW,
        WR_DATA,
        WR_RESP
    } wr_state_t;

    wr_state_t                    wr_state;
    logic [63:0]                  wr_cur_addr;
    logic [31:0]                  wr_plan_rem_beats;
    logic [C_M_AXI_ADDR_WIDTH-1:0] wr_awaddr;
    logic [7:0]                   wr_awlen;
    logic [8:0]                   wr_burst_left;
    logic                         wr_done_reg;
    logic                         axi_wr_err_reg;

    wire w_fire = m_axi_gmem_WVALID && m_axi_gmem_WREADY;
    wire b_fire = m_axi_gmem_BVALID && m_axi_gmem_BREADY;

    generate
        if (C_M_AXI_DATA_WIDTH == 64) begin : g_wr_64
            logic        wr_hi_pending;
            logic [63:0] wr_hi_data;
            logic [7:0]  wr_hi_strb;

            assign m_axi_gmem_WVALID = !reset && (wr_state == WR_DATA)
                                    && (wr_hi_pending ? 1'b1 : mem_wr_data_valid);
            assign m_axi_gmem_WDATA  = reset ? 64'd0
                                     : (wr_hi_pending ? wr_hi_data : mem_wr_data[63:0]);
            assign m_axi_gmem_WSTRB  = reset ? 8'd0
                                     : (wr_hi_pending ? wr_hi_strb : mem_wr_data_strb[7:0]);
            assign mem_wr_data_ready = !reset && (wr_state == WR_DATA)
                                    && !wr_hi_pending && m_axi_gmem_WREADY;

            always_ff @(posedge clock) begin
                if (reset) begin
                    wr_hi_pending <= 1'b0;
                    wr_hi_data    <= 64'd0;
                    wr_hi_strb    <= 8'd0;
                end else if (wr_state == WR_IDLE) begin
                    wr_hi_pending <= 1'b0;
                end else if (w_fire) begin
                    if (!wr_hi_pending) begin
                        wr_hi_pending <= 1'b1;
                        wr_hi_data    <= mem_wr_data[127:64];
                        wr_hi_strb    <= mem_wr_data_strb[15:8];
                    end else begin
                        wr_hi_pending <= 1'b0;
                    end
                end
            end
        end else begin : g_wr_128
            assign m_axi_gmem_WVALID = !reset && (wr_state == WR_DATA) && mem_wr_data_valid;
            assign m_axi_gmem_WDATA  = reset ? '0 : mem_wr_data;
            assign m_axi_gmem_WSTRB  = reset ? '0 : mem_wr_data_strb;
            assign mem_wr_data_ready = !reset && (wr_state == WR_DATA) && m_axi_gmem_WREADY;
        end
    endgenerate

    wire [8:0] wr_next_burst = calc_burst_beats(wr_cur_addr[11:0], wr_plan_rem_beats);

    always_ff @(posedge clock) begin
        if (reset) begin
            wr_state          <= WR_IDLE;
            wr_cur_addr       <= 64'd0;
            wr_plan_rem_beats <= 32'd0;
            wr_awaddr         <= '0;
            wr_awlen          <= 8'd0;
            wr_burst_left     <= 9'd0;
            wr_done_reg       <= 1'b0;
            axi_wr_err_reg    <= 1'b0;
        end else begin
            wr_done_reg    <= 1'b0;
            axi_wr_err_reg <= 1'b0;

            case (wr_state)
                WR_IDLE: begin
                    if (mem_wr_cmd_valid && mem_wr_cmd_ready) begin
                        wr_cur_addr       <= data_base_addr + 64'(mem_wr_cmd_addr);
                        wr_plan_rem_beats <= 32'(mem_wr_cmd_len) * 32'(SUBBEATS);
                        wr_state          <= WR_CALC;
                    end
                end
                WR_CALC: begin
                    wr_awaddr         <= C_M_AXI_ADDR_WIDTH'(wr_cur_addr);
                    wr_awlen          <= 8'(wr_next_burst - 9'd1);
                    wr_burst_left     <= wr_next_burst;
                    wr_cur_addr       <= wr_cur_addr + (64'(wr_next_burst) << AXI_BEAT_SHIFT);
                    wr_plan_rem_beats <= wr_plan_rem_beats - 32'(wr_next_burst);
                    wr_state          <= WR_AW;
                end
                WR_AW: begin
                    if (m_axi_gmem_AWREADY) begin
                        wr_state <= WR_DATA;
                    end
                end
                WR_DATA: begin
                    if (w_fire) begin
                        if (wr_burst_left == 9'd1) begin
                            wr_state <= WR_RESP;
                        end else begin
                            wr_burst_left <= wr_burst_left - 9'd1;
                        end
                    end
                end
                WR_RESP: begin
                    if (b_fire) begin
                        if (m_axi_gmem_BRESP != 2'b00) begin
                            axi_wr_err_reg <= 1'b1;
                        end
                        if (wr_plan_rem_beats == 32'd0) begin
                            wr_done_reg <= 1'b1;
                            wr_state    <= WR_IDLE;
                        end else begin
                            wr_state <= WR_CALC;
                        end
                    end
                end
                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    assign mem_wr_cmd_ready    = !reset && (wr_state == WR_IDLE) && !wr_done_reg;
    assign mem_wr_done         = !reset && wr_done_reg;

    assign m_axi_gmem_AWVALID  = !reset && (wr_state == WR_AW);
    assign m_axi_gmem_AWADDR   = wr_awaddr;
    assign m_axi_gmem_AWID     = '0;
    assign m_axi_gmem_AWUSER   = C_M_AXI_AWUSER_WIDTH'(AXI_COHERENT);
    assign m_axi_gmem_AWLEN    = wr_awlen;
    assign m_axi_gmem_AWSIZE   = AXI_SIZE_CONST;
    assign m_axi_gmem_AWBURST  = AXI_BURST_INCR;
    assign m_axi_gmem_AWLOCK   = AXI_LOCK_NORMAL;
    assign m_axi_gmem_AWCACHE  = AXI_CACHE_CONST;
    assign m_axi_gmem_AWPROT   = AXI_PROT_CONST;
    assign m_axi_gmem_AWQOS    = AXI_QOS_CONST;
    assign m_axi_gmem_AWREGION = AXI_REGION_CONST;

    assign m_axi_gmem_WLAST    = !reset && (wr_state == WR_DATA) && (wr_burst_left == 9'd1);
    assign m_axi_gmem_WID      = '0;
    assign m_axi_gmem_WUSER    = C_M_AXI_WUSER_WIDTH'(AXI_COHERENT);

    assign m_axi_gmem_BREADY   = !reset && (wr_state == WR_RESP);

    assign dma_busy            = !reset && ((rd_state != RD_IDLE) || rd_hold_valid
                                         || (wr_state != WR_IDLE) || wr_done_reg);
    assign axi_rd_err          = !reset && axi_rd_err_reg;
    assign axi_wr_err          = !reset && axi_wr_err_reg;

    wire _unused_axi_aux = &{
        1'b0,
        rd_cur_addr[63:12],
        wr_cur_addr[63:12],
        m_axi_gmem_BID,
        m_axi_gmem_BUSER,
        m_axi_gmem_RID,
        m_axi_gmem_RUSER
    };

`ifndef SYNTHESIS
    logic [DMA_LEN_WIDTH-1:0] dbg_wr_core_left;

    always_ff @(posedge clock) begin
        if (reset) begin
            dbg_wr_core_left <= '0;
        end else if (mem_wr_cmd_valid && mem_wr_cmd_ready) begin
            dbg_wr_core_left <= mem_wr_cmd_len;
        end else if (mem_wr_data_valid && mem_wr_data_ready) begin
            dbg_wr_core_left <= dbg_wr_core_left - DMA_LEN_WIDTH'(1);
        end
    end

    initial begin
        assert (C_M_AXI_ADDR_WIDTH == 32 || C_M_AXI_ADDR_WIDTH == 64)
            else $fatal(1, "C_M_AXI_ADDR_WIDTH must be 32 or 64");
        assert (C_M_AXI_DATA_WIDTH == 64 || C_M_AXI_DATA_WIDTH == 128)
            else $fatal(1, "C_M_AXI_DATA_WIDTH must be 64 or 128");
        assert (DMA_DATA_WIDTH == 128)
            else $fatal(1, "bnn_soc_dma expects core DMA_DATA_WIDTH == 128");
        assert (AXI_MAX_BURST_LEN >= 1 && AXI_MAX_BURST_LEN <= 256)
            else $fatal(1, "AXI_MAX_BURST_LEN must be in 1..256");
    end

    always @(posedge clock) begin
        if (!reset) begin
            if (mem_rd_cmd_valid && mem_rd_cmd_ready) begin
                assert (mem_rd_cmd_len != '0)
                    else $fatal(1, "bnn_soc_dma: mem_rd_cmd_len must be > 0");
                assert (((data_base_addr + 64'(mem_rd_cmd_addr)) % 64'(AXI_BEAT_BYTES)) == 64'd0)
                    else $fatal(1, "bnn_soc_dma: relocated read address is not AXI-beat aligned");
            end
            if (mem_wr_cmd_valid && mem_wr_cmd_ready) begin
                assert (mem_wr_cmd_len != '0)
                    else $fatal(1, "bnn_soc_dma: mem_wr_cmd_len must be > 0");
                assert (((data_base_addr + 64'(mem_wr_cmd_addr)) % 64'(AXI_BEAT_BYTES)) == 64'd0)
                    else $fatal(1, "bnn_soc_dma: relocated write address is not AXI-beat aligned");
            end
            if (r_fire) begin
                assert (m_axi_gmem_RLAST == (rd_burst_left == 9'd1))
                    else $fatal(1, "bnn_soc_dma: RLAST mismatch against commanded ARLEN");
            end
            if (mem_wr_data_valid && mem_wr_data_ready) begin
                assert (mem_wr_data_last == (dbg_wr_core_left == DMA_LEN_WIDTH'(1)))
                    else $fatal(1, "bnn_soc_dma: mem_wr_data_last mismatch against mem_wr_cmd_len");
            end
        end
    end
`else
    wire _unused_synth_ok = &{1'b0, m_axi_gmem_RLAST, mem_wr_data_last};
`endif

endmodule

`default_nettype wire
