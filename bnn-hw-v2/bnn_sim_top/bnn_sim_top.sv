`default_nettype none

/*
============================================================================
bnn_sim_top (BNN v2 面向 SoC 顶层的 Verilator 统一仿真顶层)
----------------------------------------------------------------------------
将真实的 Zynq SoC 接入顶层 bnn_soc_top 置于仿真中心，宿主机通过标准 AXI4-Lite
读写控制寄存器（s_axi_control_*），数据访存全程穿过 64-bit AXI4 Master 总线
（m_axi_gmem_*）：

  bnn_sim_cmd_stream  -- DPI mmio_*()   --> s_axi_control_* (32-bit AXI4-Lite)
  bnn_soc_top         -- 真实 SoC 顶层  --> bnn_soc_ctrl + bnn_core_top + bnn_soc_dma
  bnn_sim_data_memory <-- DPI data_*()  <-- m_axi_gmem_*    (64-bit AXI4 Full)

finish 由 testbench 在确认所有宿主机 MMIO 交互均已结束后拉高；当无在途 MMIO 事务且
SoC/AXI 从机全部空闲时自动 $finish。
============================================================================
*/
module bnn_sim_top #(
    parameter int unsigned C_S_AXI_CONTROL_ADDR_WIDTH = 16,
    parameter int unsigned C_S_AXI_CONTROL_DATA_WIDTH = 32,
    parameter int unsigned C_M_AXI_GMEM_ID_WIDTH      = 8,
    parameter int unsigned C_M_AXI_GMEM_ADDR_WIDTH    = 32,
    parameter int unsigned C_M_AXI_GMEM_DATA_WIDTH    = 64
) (
    input  wire clock,
    input  wire reset,
    input  wire finish,

    output wire busy_out,
    output wire command_active,
    output wire cmd_done_out,
    output wire cmd_error_out,
    output wire error_out,
    output wire irq_out
);
    // ------------------------------------------------------------------------
    // AXI4-Lite 控制总线连线 (bnn_sim_cmd_stream -> bnn_soc_top)
    // ------------------------------------------------------------------------
    wire                                       s_axi_control_AWVALID;
    wire                                       s_axi_control_AWREADY;
    wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]      s_axi_control_AWADDR;
    wire                                       s_axi_control_WVALID;
    wire                                       s_axi_control_WREADY;
    wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]      s_axi_control_WDATA;
    wire [(C_S_AXI_CONTROL_DATA_WIDTH/8)-1:0]  s_axi_control_WSTRB;
    wire                                       s_axi_control_BVALID;
    wire                                       s_axi_control_BREADY;
    wire [1:0]                                 s_axi_control_BRESP;
    wire                                       s_axi_control_ARVALID;
    wire                                       s_axi_control_ARREADY;
    wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]      s_axi_control_ARADDR;
    wire                                       s_axi_control_RVALID;
    wire                                       s_axi_control_RREADY;
    wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]      s_axi_control_RDATA;
    wire [1:0]                                 s_axi_control_RRESP;

    // ------------------------------------------------------------------------
    // AXI4 数据总线连线 (bnn_soc_top -> bnn_sim_data_memory)
    // ------------------------------------------------------------------------
    wire                                       m_axi_gmem_AWVALID;
    wire                                       m_axi_gmem_AWREADY;
    wire [C_M_AXI_GMEM_ADDR_WIDTH-1:0]         m_axi_gmem_AWADDR;
    wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_AWID;
    wire [0:0]                                 m_axi_gmem_AWUSER;
    wire [7:0]                                 m_axi_gmem_AWLEN;
    wire [2:0]                                 m_axi_gmem_AWSIZE;
    wire [1:0]                                 m_axi_gmem_AWBURST;
    wire [1:0]                                 m_axi_gmem_AWLOCK;
    wire [3:0]                                 m_axi_gmem_AWCACHE;
    wire [2:0]                                 m_axi_gmem_AWPROT;
    wire [3:0]                                 m_axi_gmem_AWQOS;
    wire [3:0]                                 m_axi_gmem_AWREGION;

    wire                                       m_axi_gmem_WVALID;
    wire                                       m_axi_gmem_WREADY;
    wire [C_M_AXI_GMEM_DATA_WIDTH-1:0]         m_axi_gmem_WDATA;
    wire [(C_M_AXI_GMEM_DATA_WIDTH/8)-1:0]     m_axi_gmem_WSTRB;
    wire                                       m_axi_gmem_WLAST;
    wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_WID;
    wire [0:0]                                 m_axi_gmem_WUSER;

    wire                                       m_axi_gmem_BVALID;
    wire                                       m_axi_gmem_BREADY;
    wire [1:0]                                 m_axi_gmem_BRESP;
    wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_BID;
    wire [0:0]                                 m_axi_gmem_BUSER;

    wire                                       m_axi_gmem_ARVALID;
    wire                                       m_axi_gmem_ARREADY;
    wire [C_M_AXI_GMEM_ADDR_WIDTH-1:0]         m_axi_gmem_ARADDR;
    wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_ARID;
    wire [0:0]                                 m_axi_gmem_ARUSER;
    wire [7:0]                                 m_axi_gmem_ARLEN;
    wire [2:0]                                 m_axi_gmem_ARSIZE;
    wire [1:0]                                 m_axi_gmem_ARBURST;
    wire [1:0]                                 m_axi_gmem_ARLOCK;
    wire [3:0]                                 m_axi_gmem_ARCACHE;
    wire [2:0]                                 m_axi_gmem_ARPROT;
    wire [3:0]                                 m_axi_gmem_ARQOS;
    wire [3:0]                                 m_axi_gmem_ARREGION;

    wire                                       m_axi_gmem_RVALID;
    wire                                       m_axi_gmem_RREADY;
    wire [C_M_AXI_GMEM_DATA_WIDTH-1:0]         m_axi_gmem_RDATA;
    wire [1:0]                                 m_axi_gmem_RRESP;
    wire                                       m_axi_gmem_RLAST;
    wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_RID;
    wire [0:0]                                 m_axi_gmem_RUSER;

    wire rd_busy;
    wire wr_busy;
    wire done;
    wire irq;

    // 仿真白盒观测信号（不影响可综合 bnn_soc_top 端口）
    wire soc_busy   = u_soc.u_soc_ctrl.busy;
    wire busy       = !reset && (soc_busy || rd_busy || wr_busy);

    assign busy_out      = busy;
    assign cmd_done_out  = !reset && u_soc.core_cmd_done;
    assign cmd_error_out = !reset && u_soc.core_cmd_error;
    assign error_out     = !reset && u_soc.u_soc_ctrl.error_flag;
    assign irq_out       = irq;

    bnn_sim_cmd_stream #(
        .C_S_AXI_ADDR_WIDTH (C_S_AXI_CONTROL_ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH (C_S_AXI_CONTROL_DATA_WIDTH)
    ) u_cmd_stream (
        .clock                 (clock),
        .reset                 (reset),
        .soc_busy              (busy),
        .allow_finish          (finish),
        .mmio_active           (command_active),
        .done                  (done),
        .s_axi_control_AWVALID (s_axi_control_AWVALID),
        .s_axi_control_AWREADY (s_axi_control_AWREADY),
        .s_axi_control_AWADDR  (s_axi_control_AWADDR),
        .s_axi_control_WVALID  (s_axi_control_WVALID),
        .s_axi_control_WREADY  (s_axi_control_WREADY),
        .s_axi_control_WDATA   (s_axi_control_WDATA),
        .s_axi_control_WSTRB   (s_axi_control_WSTRB),
        .s_axi_control_BVALID  (s_axi_control_BVALID),
        .s_axi_control_BREADY  (s_axi_control_BREADY),
        .s_axi_control_BRESP   (s_axi_control_BRESP),
        .s_axi_control_ARVALID (s_axi_control_ARVALID),
        .s_axi_control_ARREADY (s_axi_control_ARREADY),
        .s_axi_control_ARADDR  (s_axi_control_ARADDR),
        .s_axi_control_RVALID  (s_axi_control_RVALID),
        .s_axi_control_RREADY  (s_axi_control_RREADY),
        .s_axi_control_RDATA   (s_axi_control_RDATA),
        .s_axi_control_RRESP   (s_axi_control_RRESP)
    );

    bnn_soc_top #(
        .C_S_AXI_CONTROL_ADDR_WIDTH (C_S_AXI_CONTROL_ADDR_WIDTH),
        .C_S_AXI_CONTROL_DATA_WIDTH (C_S_AXI_CONTROL_DATA_WIDTH),
        .C_M_AXI_GMEM_ID_WIDTH      (C_M_AXI_GMEM_ID_WIDTH),
        .C_M_AXI_GMEM_ADDR_WIDTH    (C_M_AXI_GMEM_ADDR_WIDTH),
        .C_M_AXI_GMEM_DATA_WIDTH    (C_M_AXI_GMEM_DATA_WIDTH)
    ) u_soc (
        .ap_clk                (clock),
        .ap_rst_n              (~reset),
        .irq                   (irq),
        .m_axi_gmem_AWVALID    (m_axi_gmem_AWVALID),
        .m_axi_gmem_AWREADY    (m_axi_gmem_AWREADY),
        .m_axi_gmem_AWADDR     (m_axi_gmem_AWADDR),
        .m_axi_gmem_AWID       (m_axi_gmem_AWID),
        .m_axi_gmem_AWUSER     (m_axi_gmem_AWUSER),
        .m_axi_gmem_AWLEN      (m_axi_gmem_AWLEN),
        .m_axi_gmem_AWSIZE     (m_axi_gmem_AWSIZE),
        .m_axi_gmem_AWBURST    (m_axi_gmem_AWBURST),
        .m_axi_gmem_AWLOCK     (m_axi_gmem_AWLOCK),
        .m_axi_gmem_AWCACHE    (m_axi_gmem_AWCACHE),
        .m_axi_gmem_AWPROT     (m_axi_gmem_AWPROT),
        .m_axi_gmem_AWQOS      (m_axi_gmem_AWQOS),
        .m_axi_gmem_AWREGION   (m_axi_gmem_AWREGION),
        .m_axi_gmem_WVALID     (m_axi_gmem_WVALID),
        .m_axi_gmem_WREADY     (m_axi_gmem_WREADY),
        .m_axi_gmem_WDATA      (m_axi_gmem_WDATA),
        .m_axi_gmem_WSTRB      (m_axi_gmem_WSTRB),
        .m_axi_gmem_WLAST      (m_axi_gmem_WLAST),
        .m_axi_gmem_WID        (m_axi_gmem_WID),
        .m_axi_gmem_WUSER      (m_axi_gmem_WUSER),
        .m_axi_gmem_BVALID     (m_axi_gmem_BVALID),
        .m_axi_gmem_BREADY     (m_axi_gmem_BREADY),
        .m_axi_gmem_BRESP      (m_axi_gmem_BRESP),
        .m_axi_gmem_BID        (m_axi_gmem_BID),
        .m_axi_gmem_BUSER      (m_axi_gmem_BUSER),
        .m_axi_gmem_ARVALID    (m_axi_gmem_ARVALID),
        .m_axi_gmem_ARREADY    (m_axi_gmem_ARREADY),
        .m_axi_gmem_ARADDR     (m_axi_gmem_ARADDR),
        .m_axi_gmem_ARID       (m_axi_gmem_ARID),
        .m_axi_gmem_ARUSER     (m_axi_gmem_ARUSER),
        .m_axi_gmem_ARLEN      (m_axi_gmem_ARLEN),
        .m_axi_gmem_ARSIZE     (m_axi_gmem_ARSIZE),
        .m_axi_gmem_ARBURST    (m_axi_gmem_ARBURST),
        .m_axi_gmem_ARLOCK     (m_axi_gmem_ARLOCK),
        .m_axi_gmem_ARCACHE    (m_axi_gmem_ARCACHE),
        .m_axi_gmem_ARPROT     (m_axi_gmem_ARPROT),
        .m_axi_gmem_ARQOS      (m_axi_gmem_ARQOS),
        .m_axi_gmem_ARREGION   (m_axi_gmem_ARREGION),
        .m_axi_gmem_RVALID     (m_axi_gmem_RVALID),
        .m_axi_gmem_RREADY     (m_axi_gmem_RREADY),
        .m_axi_gmem_RDATA      (m_axi_gmem_RDATA),
        .m_axi_gmem_RRESP      (m_axi_gmem_RRESP),
        .m_axi_gmem_RLAST      (m_axi_gmem_RLAST),
        .m_axi_gmem_RID        (m_axi_gmem_RID),
        .m_axi_gmem_RUSER      (m_axi_gmem_RUSER),
        .s_axi_control_AWVALID (s_axi_control_AWVALID),
        .s_axi_control_AWREADY (s_axi_control_AWREADY),
        .s_axi_control_AWADDR  (s_axi_control_AWADDR),
        .s_axi_control_WVALID  (s_axi_control_WVALID),
        .s_axi_control_WREADY  (s_axi_control_WREADY),
        .s_axi_control_WDATA   (s_axi_control_WDATA),
        .s_axi_control_WSTRB   (s_axi_control_WSTRB),
        .s_axi_control_BVALID  (s_axi_control_BVALID),
        .s_axi_control_BREADY  (s_axi_control_BREADY),
        .s_axi_control_BRESP   (s_axi_control_BRESP),
        .s_axi_control_ARVALID (s_axi_control_ARVALID),
        .s_axi_control_ARREADY (s_axi_control_ARREADY),
        .s_axi_control_ARADDR  (s_axi_control_ARADDR),
        .s_axi_control_RVALID  (s_axi_control_RVALID),
        .s_axi_control_RREADY  (s_axi_control_RREADY),
        .s_axi_control_RDATA   (s_axi_control_RDATA),
        .s_axi_control_RRESP   (s_axi_control_RRESP)
    );

    bnn_sim_data_memory #(
        .C_M_AXI_ID_WIDTH   (C_M_AXI_GMEM_ID_WIDTH),
        .C_M_AXI_ADDR_WIDTH (C_M_AXI_GMEM_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH (C_M_AXI_GMEM_DATA_WIDTH)
    ) u_data_memory (
        .clock              (clock),
        .reset              (reset),
        .m_axi_gmem_AWVALID (m_axi_gmem_AWVALID),
        .m_axi_gmem_AWREADY (m_axi_gmem_AWREADY),
        .m_axi_gmem_AWADDR  (m_axi_gmem_AWADDR),
        .m_axi_gmem_AWID    (m_axi_gmem_AWID),
        .m_axi_gmem_AWUSER  (m_axi_gmem_AWUSER),
        .m_axi_gmem_AWLEN   (m_axi_gmem_AWLEN),
        .m_axi_gmem_AWSIZE  (m_axi_gmem_AWSIZE),
        .m_axi_gmem_AWBURST (m_axi_gmem_AWBURST),
        .m_axi_gmem_AWLOCK  (m_axi_gmem_AWLOCK),
        .m_axi_gmem_AWCACHE (m_axi_gmem_AWCACHE),
        .m_axi_gmem_AWPROT  (m_axi_gmem_AWPROT),
        .m_axi_gmem_AWQOS   (m_axi_gmem_AWQOS),
        .m_axi_gmem_AWREGION(m_axi_gmem_AWREGION),
        .m_axi_gmem_WVALID  (m_axi_gmem_WVALID),
        .m_axi_gmem_WREADY  (m_axi_gmem_WREADY),
        .m_axi_gmem_WDATA   (m_axi_gmem_WDATA),
        .m_axi_gmem_WSTRB   (m_axi_gmem_WSTRB),
        .m_axi_gmem_WLAST   (m_axi_gmem_WLAST),
        .m_axi_gmem_WID     (m_axi_gmem_WID),
        .m_axi_gmem_WUSER   (m_axi_gmem_WUSER),
        .m_axi_gmem_BVALID  (m_axi_gmem_BVALID),
        .m_axi_gmem_BREADY  (m_axi_gmem_BREADY),
        .m_axi_gmem_BRESP   (m_axi_gmem_BRESP),
        .m_axi_gmem_BID     (m_axi_gmem_BID),
        .m_axi_gmem_BUSER   (m_axi_gmem_BUSER),
        .m_axi_gmem_ARVALID (m_axi_gmem_ARVALID),
        .m_axi_gmem_ARREADY (m_axi_gmem_ARREADY),
        .m_axi_gmem_ARADDR  (m_axi_gmem_ARADDR),
        .m_axi_gmem_ARID    (m_axi_gmem_ARID),
        .m_axi_gmem_ARUSER  (m_axi_gmem_ARUSER),
        .m_axi_gmem_ARLEN   (m_axi_gmem_ARLEN),
        .m_axi_gmem_ARSIZE  (m_axi_gmem_ARSIZE),
        .m_axi_gmem_ARBURST (m_axi_gmem_ARBURST),
        .m_axi_gmem_ARLOCK  (m_axi_gmem_ARLOCK),
        .m_axi_gmem_ARCACHE (m_axi_gmem_ARCACHE),
        .m_axi_gmem_ARPROT  (m_axi_gmem_ARPROT),
        .m_axi_gmem_ARQOS   (m_axi_gmem_ARQOS),
        .m_axi_gmem_ARREGION(m_axi_gmem_ARREGION),
        .m_axi_gmem_RVALID  (m_axi_gmem_RVALID),
        .m_axi_gmem_RREADY  (m_axi_gmem_RREADY),
        .m_axi_gmem_RDATA   (m_axi_gmem_RDATA),
        .m_axi_gmem_RRESP   (m_axi_gmem_RRESP),
        .m_axi_gmem_RLAST   (m_axi_gmem_RLAST),
        .m_axi_gmem_RID     (m_axi_gmem_RID),
        .m_axi_gmem_RUSER   (m_axi_gmem_RUSER),
        .rd_busy            (rd_busy),
        .wr_busy            (wr_busy)
    );

    always @(posedge clock) begin
        if (!reset && done) begin
            $display("bnn_sim_top: all commands completed");
            $finish;
        end
    end
endmodule

`default_nettype wire
