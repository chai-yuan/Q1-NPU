`default_nettype none

/*
============================================================================
bnn_sim_data_memory (64-bit AXI4 Slave 统一数据存储器 DPI-C 适配层)
----------------------------------------------------------------------------
- 挂载于 bnn_soc_top 的 m_axi_gmem_* 接口，模拟 Zynq PS S_AXI_ACP / S_AXI_HP0
  后端的 DDR 存储系统，同时对 AXI4 突发协议（INCR、8B 对齐、4KiB 边界不跨越、
  WLAST 拍数一致性）进行实时断言校验。
- 复用原有 64-bit 小端字节地址 DPI-C 存储契约：
    longint unsigned data_read (longint unsigned address);
    void data_read_burst(longint unsigned address, longint unsigned bytes);
    void data_write(longint unsigned address, longint unsigned data,
                    byte unsigned strobe);
- 读通道：AR 握手时先调用一次 data_read_burst() 预取整笔 AXI 突发（使 TCP 协同
  仿真整包拉取数据），随后逐拍返回 64-bit RDATA 并在末拍置位 RLAST。
- 写通道：AW 握手锁存突发信息，W 握手逐拍调用 data_write()，收齐末拍 WLAST 后
  在下一拍返回 BVALID (BRESP = OKAY)。
============================================================================
*/
module bnn_sim_data_memory #(
    parameter int unsigned C_M_AXI_ID_WIDTH     = 8,
    parameter int unsigned C_M_AXI_ADDR_WIDTH   = 32,
    parameter int unsigned C_M_AXI_DATA_WIDTH   = 64,
    parameter int unsigned C_M_AXI_AWUSER_WIDTH = 1,
    parameter int unsigned C_M_AXI_ARUSER_WIDTH = 1,
    parameter int unsigned C_M_AXI_WUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_RUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_BUSER_WIDTH  = 1
) (
    input  wire                                  clock,
    input  wire                                  reset,

    // AXI4 Slave 接口（对接 bnn_soc_top.m_axi_gmem_*）
    input  wire                                  m_axi_gmem_AWVALID,
    output wire                                  m_axi_gmem_AWREADY,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0]         m_axi_gmem_AWADDR,
    input  wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_AWID,
    input  wire [C_M_AXI_AWUSER_WIDTH-1:0]       m_axi_gmem_AWUSER,
    input  wire [7:0]                            m_axi_gmem_AWLEN,
    input  wire [2:0]                            m_axi_gmem_AWSIZE,
    input  wire [1:0]                            m_axi_gmem_AWBURST,
    input  wire [1:0]                            m_axi_gmem_AWLOCK,
    input  wire [3:0]                            m_axi_gmem_AWCACHE,
    input  wire [2:0]                            m_axi_gmem_AWPROT,
    input  wire [3:0]                            m_axi_gmem_AWQOS,
    input  wire [3:0]                            m_axi_gmem_AWREGION,

    input  wire                                  m_axi_gmem_WVALID,
    output wire                                  m_axi_gmem_WREADY,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]         m_axi_gmem_WDATA,
    input  wire [(C_M_AXI_DATA_WIDTH/8)-1:0]     m_axi_gmem_WSTRB,
    input  wire                                  m_axi_gmem_WLAST,
    input  wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_WID,
    input  wire [C_M_AXI_WUSER_WIDTH-1:0]        m_axi_gmem_WUSER,

    output wire                                  m_axi_gmem_BVALID,
    input  wire                                  m_axi_gmem_BREADY,
    output wire [1:0]                            m_axi_gmem_BRESP,
    output wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_BID,
    output wire [C_M_AXI_BUSER_WIDTH-1:0]        m_axi_gmem_BUSER,

    input  wire                                  m_axi_gmem_ARVALID,
    output wire                                  m_axi_gmem_ARREADY,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0]         m_axi_gmem_ARADDR,
    input  wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_ARID,
    input  wire [C_M_AXI_ARUSER_WIDTH-1:0]       m_axi_gmem_ARUSER,
    input  wire [7:0]                            m_axi_gmem_ARLEN,
    input  wire [2:0]                            m_axi_gmem_ARSIZE,
    input  wire [1:0]                            m_axi_gmem_ARBURST,
    input  wire [1:0]                            m_axi_gmem_ARLOCK,
    input  wire [3:0]                            m_axi_gmem_ARCACHE,
    input  wire [2:0]                            m_axi_gmem_ARPROT,
    input  wire [3:0]                            m_axi_gmem_ARQOS,
    input  wire [3:0]                            m_axi_gmem_ARREGION,

    output wire                                  m_axi_gmem_RVALID,
    input  wire                                  m_axi_gmem_RREADY,
    output wire [C_M_AXI_DATA_WIDTH-1:0]         m_axi_gmem_RDATA,
    output wire [1:0]                            m_axi_gmem_RRESP,
    output wire                                  m_axi_gmem_RLAST,
    output wire [C_M_AXI_ID_WIDTH-1:0]           m_axi_gmem_RID,
    output wire [C_M_AXI_RUSER_WIDTH-1:0]        m_axi_gmem_RUSER,

    output wire                                  rd_busy,
    output wire                                  wr_busy
);
    import "DPI-C" function longint unsigned data_read(
        input longint unsigned address);
    import "DPI-C" function void data_read_burst(
        input longint unsigned address, input longint unsigned bytes);
    import "DPI-C" function void data_write(
        input longint unsigned address, input longint unsigned data,
        input byte unsigned strobe);

    localparam int unsigned BEAT_BYTES = C_M_AXI_DATA_WIDTH / 8;
    localparam int unsigned WORDS64    = C_M_AXI_DATA_WIDTH / 64;
    localparam logic [2:0]  EXP_SIZE   = 3'($clog2(BEAT_BYTES));

    typedef enum logic {RD_IDLE, RD_DATA} rd_state_t;
    typedef enum logic [1:0] {WR_IDLE, WR_DATA, WR_RESP} wr_state_t;

    rd_state_t                    rd_state;
    logic [63:0]                  rd_address;
    logic [8:0]                   rd_remaining;
    logic [C_M_AXI_ID_WIDTH-1:0]  rd_id_reg;
    logic                         r_valid_reg;
    logic [C_M_AXI_DATA_WIDTH-1:0] r_data_reg;
    logic                         r_last_reg;

    wr_state_t                    wr_state;
    logic [63:0]                  wr_address;
    logic [8:0]                   wr_remaining;
    logic [C_M_AXI_ID_WIDTH-1:0]  wr_id_reg;

    assign m_axi_gmem_ARREADY = !reset && (rd_state == RD_IDLE);
    assign m_axi_gmem_RVALID  = !reset && r_valid_reg;
    assign m_axi_gmem_RDATA   = reset ? '0 : r_data_reg;
    assign m_axi_gmem_RRESP   = 2'b00;
    assign m_axi_gmem_RLAST   = !reset && r_last_reg;
    assign m_axi_gmem_RID     = rd_id_reg;
    assign m_axi_gmem_RUSER   = '0;
    assign rd_busy            = !reset && (rd_state != RD_IDLE);

    assign m_axi_gmem_AWREADY = !reset && (wr_state == WR_IDLE);
    assign m_axi_gmem_WREADY  = !reset && (wr_state == WR_DATA);
    assign m_axi_gmem_BVALID  = !reset && (wr_state == WR_RESP);
    assign m_axi_gmem_BRESP   = 2'b00;
    assign m_axi_gmem_BID     = wr_id_reg;
    assign m_axi_gmem_BUSER   = '0;
    assign wr_busy            = !reset && (wr_state != WR_IDLE);

    task automatic present_axi_read(input logic [63:0] address, input logic last);
        logic [C_M_AXI_DATA_WIDTH-1:0] packed_data;
        for (int w = 0; w < WORDS64; w++)
            packed_data[64*w +: 64] = 64'(data_read(address + 64'(8*w)));
        r_data_reg  <= packed_data;
        r_last_reg  <= last;
        r_valid_reg <= 1'b1;
    endtask

    // ------------------------------------------------------------------------
    // AXI4 读从机时序与 4KiB 边界检查
    // ------------------------------------------------------------------------
    always_ff @(posedge clock) begin
        if (reset) begin
            rd_state     <= RD_IDLE;
            rd_address   <= 64'd0;
            rd_remaining <= 9'd0;
            rd_id_reg    <= '0;
            r_valid_reg  <= 1'b0;
            r_data_reg   <= '0;
            r_last_reg   <= 1'b0;
        end else begin
            case (rd_state)
                RD_IDLE: if (m_axi_gmem_ARVALID && m_axi_gmem_ARREADY) begin
                    logic [63:0] req_beats;
                    logic [63:0] req_bytes;
                    req_beats = 64'(m_axi_gmem_ARLEN) + 64'd1;
                    req_bytes = req_beats * 64'(BEAT_BYTES);
                    if (m_axi_gmem_ARBURST != 2'b01)
                        $fatal(1, "AXI ARBURST must be INCR (2'b01)");
                    if (m_axi_gmem_ARSIZE != EXP_SIZE)
                        $fatal(1, "AXI ARSIZE does not match bus data width");
                    if ((64'(m_axi_gmem_ARADDR) % 64'(BEAT_BYTES)) != 64'd0)
                        $fatal(1, "AXI ARADDR is not beat aligned");
                    if ((64'(m_axi_gmem_ARADDR[11:0]) + req_bytes) > 64'd4096)
                        $fatal(1, "AXI read burst crosses 4KiB boundary");

                    data_read_burst(64'(m_axi_gmem_ARADDR), req_bytes);
                    rd_address   <= 64'(m_axi_gmem_ARADDR);
                    rd_remaining <= 9'(req_beats);
                    rd_id_reg    <= m_axi_gmem_ARID;
                    present_axi_read(64'(m_axi_gmem_ARADDR), m_axi_gmem_ARLEN == 8'd0);
                    rd_state     <= RD_DATA;
                end
                RD_DATA: if (r_valid_reg && m_axi_gmem_RREADY) begin
                    if (rd_remaining == 9'd1) begin
                        r_valid_reg  <= 1'b0;
                        r_last_reg   <= 1'b0;
                        rd_remaining <= 9'd0;
                        rd_state     <= RD_IDLE;
                    end else begin
                        rd_address   <= rd_address + 64'(BEAT_BYTES);
                        rd_remaining <= rd_remaining - 9'd1;
                        present_axi_read(rd_address + 64'(BEAT_BYTES), rd_remaining == 9'd2);
                    end
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // AXI4 写从机时序与 4KiB 边界检查
    // ------------------------------------------------------------------------
    always_ff @(posedge clock) begin
        if (reset) begin
            wr_state     <= WR_IDLE;
            wr_address   <= 64'd0;
            wr_remaining <= 9'd0;
            wr_id_reg    <= '0;
        end else begin
            case (wr_state)
                WR_IDLE: if (m_axi_gmem_AWVALID && m_axi_gmem_AWREADY) begin
                    logic [63:0] req_beats;
                    logic [63:0] req_bytes;
                    req_beats = 64'(m_axi_gmem_AWLEN) + 64'd1;
                    req_bytes = req_beats * 64'(BEAT_BYTES);
                    if (m_axi_gmem_AWBURST != 2'b01)
                        $fatal(1, "AXI AWBURST must be INCR (2'b01)");
                    if (m_axi_gmem_AWSIZE != EXP_SIZE)
                        $fatal(1, "AXI AWSIZE does not match bus data width");
                    if ((64'(m_axi_gmem_AWADDR) % 64'(BEAT_BYTES)) != 64'd0)
                        $fatal(1, "AXI AWADDR is not beat aligned");
                    if ((64'(m_axi_gmem_AWADDR[11:0]) + req_bytes) > 64'd4096)
                        $fatal(1, "AXI write burst crosses 4KiB boundary");

                    wr_address   <= 64'(m_axi_gmem_AWADDR);
                    wr_remaining <= 9'(req_beats);
                    wr_id_reg    <= m_axi_gmem_AWID;
                    wr_state     <= WR_DATA;
                end
                WR_DATA: if (m_axi_gmem_WVALID && m_axi_gmem_WREADY) begin
                    if (m_axi_gmem_WLAST != (wr_remaining == 9'd1))
                        $fatal(1, "AXI WLAST disagrees with commanded AWLEN");
                    for (int w = 0; w < WORDS64; w++)
                        data_write(wr_address + 64'(8*w),
                                   m_axi_gmem_WDATA[64*w +: 64],
                                   8'(m_axi_gmem_WSTRB[8*w +: 8]));
                    if (wr_remaining == 9'd1) begin
                        wr_remaining <= 9'd0;
                        wr_state     <= WR_RESP;
                    end else begin
                        wr_address   <= wr_address + 64'(BEAT_BYTES);
                        wr_remaining <= wr_remaining - 9'd1;
                    end
                end
                WR_RESP: if (m_axi_gmem_BVALID && m_axi_gmem_BREADY) begin
                    wr_state <= WR_IDLE;
                end
                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    wire _unused_axi_attrs = &{
        1'b0,
        m_axi_gmem_AWUSER,
        m_axi_gmem_AWLOCK,
        m_axi_gmem_AWCACHE,
        m_axi_gmem_AWPROT,
        m_axi_gmem_AWQOS,
        m_axi_gmem_AWREGION,
        m_axi_gmem_WID,
        m_axi_gmem_WUSER,
        m_axi_gmem_ARUSER,
        m_axi_gmem_ARLOCK,
        m_axi_gmem_ARCACHE,
        m_axi_gmem_ARPROT,
        m_axi_gmem_ARQOS,
        m_axi_gmem_ARREGION
    };

    initial begin
        assert (C_M_AXI_DATA_WIDTH == 64 || C_M_AXI_DATA_WIDTH == 128)
            else $fatal(1, "C_M_AXI_DATA_WIDTH must be 64 or 128");
    end

endmodule

`default_nettype wire
