`default_nettype none

/*
============================================================================
bnn_sim_cmd_stream (仿真宿主机 AXI4-Lite Master 总线功能模型)
----------------------------------------------------------------------------
- 通过 DPI-C 从宿主机侧（离线 C++ 驱动或远程 TCP 服务）拉取 32-bit 寄存器读写请求，
  转换为标准 AXI4-Lite Master 事务驱动 bnn_soc_top 的 s_axi_control_* 接口。
- DPI-C 契约：
    int mmio_req_check();
      非 0 表示宿主机队列中存在待执行的 MMIO 读写请求。
    longint unsigned mmio_req_get();
      取出一笔 MMIO 请求，位域打包格式：
        [31:0]  wdata    : 32-bit 写数据
        [47:32] addr     : 16-bit 字节偏移地址
        [51:48] wstrb    : 4-bit 字节写掩码
        [56]    is_write : 1 = AXI4-Lite 写事务，0 = AXI4-Lite 读事务
    void mmio_resp_put(int is_write, int unsigned rdata, int unsigned resp);
      当 AXI4-Lite B 通道（写）或 R 通道（读）完成握手时回调通知宿主机。
- allow_finish 有效、无待发 MMIO 请求且 SoC/AXI 存储空闲时拉高 done，由顶层结束仿真。
============================================================================
*/
module bnn_sim_cmd_stream #(
    parameter int unsigned C_S_AXI_ADDR_WIDTH = 16,
    parameter int unsigned C_S_AXI_DATA_WIDTH = 32
) (
    input  wire                                  clock,
    input  wire                                  reset,

    input  wire                                  soc_busy,
    input  wire                                  allow_finish,
    output wire                                  mmio_active,
    output wire                                  done,

    // AXI4-Lite Master 接口（驱动 bnn_soc_top.s_axi_control_*）
    output wire                                  s_axi_control_AWVALID,
    input  wire                                  s_axi_control_AWREADY,
    output wire [C_S_AXI_ADDR_WIDTH-1:0]         s_axi_control_AWADDR,

    output wire                                  s_axi_control_WVALID,
    input  wire                                  s_axi_control_WREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0]         s_axi_control_WDATA,
    output wire [(C_S_AXI_DATA_WIDTH/8)-1:0]     s_axi_control_WSTRB,

    input  wire                                  s_axi_control_BVALID,
    output wire                                  s_axi_control_BREADY,
    input  wire [1:0]                            s_axi_control_BRESP,

    output wire                                  s_axi_control_ARVALID,
    input  wire                                  s_axi_control_ARREADY,
    output wire [C_S_AXI_ADDR_WIDTH-1:0]         s_axi_control_ARADDR,

    input  wire                                  s_axi_control_RVALID,
    output wire                                  s_axi_control_RREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]         s_axi_control_RDATA,
    input  wire [1:0]                            s_axi_control_RRESP
);
    import "DPI-C" function int mmio_req_check();
    import "DPI-C" function longint unsigned mmio_req_get();
    import "DPI-C" function void mmio_resp_put(
        input int is_write,
        input int unsigned rdata,
        input int unsigned resp
    );

    typedef enum logic [1:0] {
        MMIO_IDLE,
        MMIO_WRITE,
        MMIO_READ
    } state_t;

    state_t                        state;
    logic [C_S_AXI_ADDR_WIDTH-1:0] addr_reg;
    logic [31:0]                   wdata_reg;
    logic [3:0]                    wstrb_reg;
    logic                          aw_valid_reg;
    logic                          w_valid_reg;
    logic                          ar_valid_reg;
    logic                          done_reg;

    assign mmio_active           = !reset && (state != MMIO_IDLE);
    assign done                  = !reset && done_reg;

    assign s_axi_control_AWVALID = !reset && aw_valid_reg;
    assign s_axi_control_AWADDR  = addr_reg;
    assign s_axi_control_WVALID  = !reset && w_valid_reg;
    assign s_axi_control_WDATA   = wdata_reg;
    assign s_axi_control_WSTRB   = wstrb_reg;
    assign s_axi_control_BREADY  = !reset && (state == MMIO_WRITE);

    assign s_axi_control_ARVALID = !reset && ar_valid_reg;
    assign s_axi_control_ARADDR  = addr_reg;
    assign s_axi_control_RREADY  = !reset && (state == MMIO_READ);

    always_ff @(posedge clock) begin
        if (reset) begin
            state        <= MMIO_IDLE;
            addr_reg     <= '0;
            wdata_reg    <= 32'd0;
            wstrb_reg    <= 4'd0;
            aw_valid_reg <= 1'b0;
            w_valid_reg  <= 1'b0;
            ar_valid_reg <= 1'b0;
            done_reg     <= 1'b0;
        end else if (!done_reg) begin
            case (state)
                MMIO_IDLE: begin
                    if (mmio_req_check() != 0) begin
                        longint unsigned req;
                        req       = mmio_req_get();
                        wdata_reg <= req[31:0];
                        addr_reg  <= C_S_AXI_ADDR_WIDTH'(req[47:32]);
                        wstrb_reg <= req[51:48];
                        if (req[56]) begin
                            aw_valid_reg <= 1'b1;
                            w_valid_reg  <= 1'b1;
                            state        <= MMIO_WRITE;
                        end else begin
                            ar_valid_reg <= 1'b1;
                            state        <= MMIO_READ;
                        end
                        if (&{1'b0, req[63:57], req[55:52]}) ;
                    end else if (!soc_busy && allow_finish) begin
                        done_reg <= 1'b1;
                    end
                end
                MMIO_WRITE: begin
                    if (aw_valid_reg && s_axi_control_AWREADY)
                        aw_valid_reg <= 1'b0;
                    if (w_valid_reg && s_axi_control_WREADY)
                        w_valid_reg <= 1'b0;
                    if (s_axi_control_BVALID && s_axi_control_BREADY) begin
                        mmio_resp_put(1, 32'd0, {30'd0, s_axi_control_BRESP});
                        state <= MMIO_IDLE;
                    end
                end
                MMIO_READ: begin
                    if (ar_valid_reg && s_axi_control_ARREADY)
                        ar_valid_reg <= 1'b0;
                    if (s_axi_control_RVALID && s_axi_control_RREADY) begin
                        mmio_resp_put(0, s_axi_control_RDATA, {30'd0, s_axi_control_RRESP});
                        state <= MMIO_IDLE;
                    end
                end
                default: state <= MMIO_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
