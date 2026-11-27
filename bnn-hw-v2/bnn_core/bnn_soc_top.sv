`default_nettype none

/*
============================================================================
bnn_soc_top (BNN v2 面向 Zynq SoC 接入与板级闭环测试的顶层封装)
----------------------------------------------------------------------------
模块职责与设计参考：
参考 Apache TVM VTA 的 Xilinx/Zynq Shell 顶层接口命名与分层思想（XilinxShell +
VCR + VMESimple），采用面向 FPGA 早期闭环测试的最简架构，将协议无关计算核心
bnn_core_top 封装为符合 Xilinx Vivado IP Integrator (IPI) 规范的标准 AXI IP：

1. 时钟、复位与中断：
   - ap_clk    ：主工作时钟（s_axi_control 与 m_axi_gmem 同源同频）
   - ap_rst_n  ：同步低有效外部复位（对接 Zynq proc_sys_reset peripheral_aresetn）
   - irq       ：高电平中断输出（可悬空采用轮询，或接 Zynq PS IRQ_F2P）
2. 宿主机控制口 s_axi_control_* (AXI4-Lite Slave, 32-bit)：
   - 对接 Zynq PS M_AXI_GP0 (Zynq-7000) 或 M_AXI_HPM0_FPD (Zynq UltraScale+)
   - 由 bnn_soc_ctrl 实现单条 64-bit 指令 MMIO 直推、状态轮询、周期计数与基址重定位
3. 全局内存口 m_axi_gmem_* (AXI4 Full Master, 默认 64-bit，可配 128-bit)：
   - 默认 64-bit 直接适配 Zynq-7000 S_AXI_ACP / S_AXI_HP0（与 VTA PynqConfig 一致）
   - 由 bnn_soc_dma 实现核心 128-bit 读写通道至 64-bit/128-bit AXI4 突发的齿轮箱拼拆、
     4KiB 地址边界切分与最大突发长度切分

内部模块拓扑：
                 s_axi_control_* (AXI4-Lite Slave, 32b)
                                   |
                                   v
                 +-----------------------------------+
   irq <---------|           bnn_soc_ctrl            |
                 |  (MMIO 单条指令直推 + 状态/计数)   |
                 +--------+-----------------+--------+
                          | cmd_* / status  | data_base / err / busy
                          v                 |
                 +------------------+       |
                 |   bnn_core_top   |       |
                 |  (BNN v2 计算核) |       |
                 +---+----------+---+       |
                     |          |           |
            mem_rd_* |          | mem_wr_*  |
                     v          v           v
                 +-----------------------------------+
                 |            bnn_soc_dma            |
                 | (128b<->64b/128b + 4KiB 突发切分) |
                 +-----------------+-----------------+
                                   |
                                   v
                   m_axi_gmem_* (AXI4 Master, 默认 64b)
============================================================================
*/
module bnn_soc_top #(
    // ------------------------------------------------------------------------
    // Xilinx Vivado IPI 标准 AXI 接口参数（遵循 C_S_AXI_* / C_M_AXI_* 命名规范）
    // ------------------------------------------------------------------------
    parameter int unsigned C_S_AXI_CONTROL_ADDR_WIDTH = 16,
    parameter int unsigned C_S_AXI_CONTROL_DATA_WIDTH = 32,

    parameter int unsigned C_M_AXI_GMEM_ID_WIDTH      = 8,
    parameter int unsigned C_M_AXI_GMEM_ADDR_WIDTH    = 32,
    parameter int unsigned C_M_AXI_GMEM_DATA_WIDTH    = 64,
    parameter int unsigned C_M_AXI_GMEM_AWUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_GMEM_ARUSER_WIDTH  = 1,
    parameter int unsigned C_M_AXI_GMEM_WUSER_WIDTH   = 1,
    parameter int unsigned C_M_AXI_GMEM_RUSER_WIDTH   = 1,
    parameter int unsigned C_M_AXI_GMEM_BUSER_WIDTH   = 1,

    // ------------------------------------------------------------------------
    // Shell 访存属性参数
    // ------------------------------------------------------------------------
    parameter bit          AXI_COHERENT               = 1'b1,
    parameter int unsigned AXI_MAX_BURST_LEN          = 256,

    // ------------------------------------------------------------------------
    // BNN v2 计算核心透传参数（与 bnn_core_top 保持一致）
    // ------------------------------------------------------------------------
    parameter int unsigned CORE_ADDR_WIDTH            = 32,
    parameter int unsigned STRIDE_WIDTH               = 32,
    parameter int unsigned DMA_DATA_WIDTH             = 128,
    parameter int unsigned DMA_LEN_WIDTH              = 16,
    parameter int unsigned IDMA_INT8_NUM              = 128,
    parameter int unsigned WEIGHT_WIDTH               = IDMA_INT8_NUM + 16,
    parameter int unsigned ODMA_FP32_NUM              = 4,
    parameter int unsigned MAX_ROWS                   = 128,
    parameter int unsigned WSRAM_ADDR_WIDTH           = 7,
    parameter int unsigned WSRAM_READ_LATENCY         = 1,
    parameter logic [WSRAM_ADDR_WIDTH-1:0] WEIGHT_BASE_ADDR = '0,
    parameter int unsigned MAX_WEIGHT_GROUPS          = (1 << WSRAM_ADDR_WIDTH) / ODMA_FP32_NUM,
    parameter int unsigned OSRAM_READ_LATENCY         = 1
) (
    // ------------------------------------------------------------------------
    // 1. 全局时钟、同步低有效复位与中断（对齐 Xilinx / VTA XilinxShell 规范）
    // ------------------------------------------------------------------------
    input  wire                                       ap_clk,
    input  wire                                       ap_rst_n,
    output wire                                       irq,

    // ------------------------------------------------------------------------
    // 2. 全局内存 AXI4 Master 接口 (m_axi_gmem_*)
    // ------------------------------------------------------------------------
    // Write Address Channel (AW)
    output wire                                       m_axi_gmem_AWVALID,
    input  wire                                       m_axi_gmem_AWREADY,
    output wire [C_M_AXI_GMEM_ADDR_WIDTH-1:0]         m_axi_gmem_AWADDR,
    output wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_AWID,
    output wire [C_M_AXI_GMEM_AWUSER_WIDTH-1:0]       m_axi_gmem_AWUSER,
    output wire [7:0]                                 m_axi_gmem_AWLEN,
    output wire [2:0]                                 m_axi_gmem_AWSIZE,
    output wire [1:0]                                 m_axi_gmem_AWBURST,
    output wire [1:0]                                 m_axi_gmem_AWLOCK,
    output wire [3:0]                                 m_axi_gmem_AWCACHE,
    output wire [2:0]                                 m_axi_gmem_AWPROT,
    output wire [3:0]                                 m_axi_gmem_AWQOS,
    output wire [3:0]                                 m_axi_gmem_AWREGION,

    // Write Data Channel (W)
    output wire                                       m_axi_gmem_WVALID,
    input  wire                                       m_axi_gmem_WREADY,
    output wire [C_M_AXI_GMEM_DATA_WIDTH-1:0]         m_axi_gmem_WDATA,
    output wire [(C_M_AXI_GMEM_DATA_WIDTH/8)-1:0]     m_axi_gmem_WSTRB,
    output wire                                       m_axi_gmem_WLAST,
    output wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_WID,
    output wire [C_M_AXI_GMEM_WUSER_WIDTH-1:0]        m_axi_gmem_WUSER,

    // Write Response Channel (B)
    input  wire                                       m_axi_gmem_BVALID,
    output wire                                       m_axi_gmem_BREADY,
    input  wire [1:0]                                 m_axi_gmem_BRESP,
    input  wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_BID,
    input  wire [C_M_AXI_GMEM_BUSER_WIDTH-1:0]        m_axi_gmem_BUSER,

    // Read Address Channel (AR)
    output wire                                       m_axi_gmem_ARVALID,
    input  wire                                       m_axi_gmem_ARREADY,
    output wire [C_M_AXI_GMEM_ADDR_WIDTH-1:0]         m_axi_gmem_ARADDR,
    output wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_ARID,
    output wire [C_M_AXI_GMEM_ARUSER_WIDTH-1:0]       m_axi_gmem_ARUSER,
    output wire [7:0]                                 m_axi_gmem_ARLEN,
    output wire [2:0]                                 m_axi_gmem_ARSIZE,
    output wire [1:0]                                 m_axi_gmem_ARBURST,
    output wire [1:0]                                 m_axi_gmem_ARLOCK,
    output wire [3:0]                                 m_axi_gmem_ARCACHE,
    output wire [2:0]                                 m_axi_gmem_ARPROT,
    output wire [3:0]                                 m_axi_gmem_ARQOS,
    output wire [3:0]                                 m_axi_gmem_ARREGION,

    // Read Data Channel (R)
    input  wire                                       m_axi_gmem_RVALID,
    output wire                                       m_axi_gmem_RREADY,
    input  wire [C_M_AXI_GMEM_DATA_WIDTH-1:0]         m_axi_gmem_RDATA,
    input  wire [1:0]                                 m_axi_gmem_RRESP,
    input  wire                                       m_axi_gmem_RLAST,
    input  wire [C_M_AXI_GMEM_ID_WIDTH-1:0]           m_axi_gmem_RID,
    input  wire [C_M_AXI_GMEM_RUSER_WIDTH-1:0]        m_axi_gmem_RUSER,

    // ------------------------------------------------------------------------
    // 3. 宿主机控制 AXI4-Lite Slave 接口 (s_axi_control_*)
    // ------------------------------------------------------------------------
    // Write Address Channel (AW)
    input  wire                                       s_axi_control_AWVALID,
    output wire                                       s_axi_control_AWREADY,
    input  wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]      s_axi_control_AWADDR,

    // Write Data Channel (W)
    input  wire                                       s_axi_control_WVALID,
    output wire                                       s_axi_control_WREADY,
    input  wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]      s_axi_control_WDATA,
    input  wire [(C_S_AXI_CONTROL_DATA_WIDTH/8)-1:0]  s_axi_control_WSTRB,

    // Write Response Channel (B)
    output wire                                       s_axi_control_BVALID,
    input  wire                                       s_axi_control_BREADY,
    output wire [1:0]                                 s_axi_control_BRESP,

    // Read Address Channel (AR)
    input  wire                                       s_axi_control_ARVALID,
    output wire                                       s_axi_control_ARREADY,
    input  wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]      s_axi_control_ARADDR,

    // Read Data Channel (R)
    output wire                                       s_axi_control_RVALID,
    input  wire                                       s_axi_control_RREADY,
    output wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]      s_axi_control_RDATA,
    output wire [1:0]                                 s_axi_control_RRESP
);
    wire clock     = ap_clk;
    wire ext_reset = ~ap_rst_n;
    wire soft_reset;
    wire core_reset = ext_reset || soft_reset;

    // ------------------------------------------------------------------------
    // bnn_soc_ctrl <-> bnn_core_top / bnn_soc_dma 内部连线
    // ------------------------------------------------------------------------
    wire                             core_cmd_valid;
    wire [63:0]                      core_cmd_inst;
    wire                             core_cmd_ready;
    wire                             core_cmd_done;
    wire                             core_cmd_error;
    wire                             core_status_busy;

    wire [63:0]                      dma_data_base_addr;
    wire                             dma_busy;
    wire                             dma_axi_rd_err;
    wire                             dma_axi_wr_err;

    wire                             core_mem_rd_cmd_valid;
    wire                             core_mem_rd_cmd_ready;
    wire [CORE_ADDR_WIDTH-1:0]       core_mem_rd_cmd_addr;
    wire [DMA_LEN_WIDTH-1:0]         core_mem_rd_cmd_len;
    wire                             core_mem_rd_data_valid;
    wire                             core_mem_rd_data_ready;
    wire [DMA_DATA_WIDTH-1:0]        core_mem_rd_data;
    wire                             core_mem_rd_data_last;

    wire                             core_mem_wr_cmd_valid;
    wire                             core_mem_wr_cmd_ready;
    wire [CORE_ADDR_WIDTH-1:0]       core_mem_wr_cmd_addr;
    wire [DMA_LEN_WIDTH-1:0]         core_mem_wr_cmd_len;
    wire                             core_mem_wr_data_valid;
    wire                             core_mem_wr_data_ready;
    wire [DMA_DATA_WIDTH-1:0]        core_mem_wr_data;
    wire [(DMA_DATA_WIDTH/8)-1:0]    core_mem_wr_data_strb;
    wire                             core_mem_wr_data_last;
    wire                             core_mem_wr_done;

    // ------------------------------------------------------------------------
    // 1. AXI4-Lite 控制寄存器与 MMIO 单条指令直推单元
    // ------------------------------------------------------------------------
    bnn_soc_ctrl #(
        .C_S_AXI_ADDR_WIDTH (C_S_AXI_CONTROL_ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH (C_S_AXI_CONTROL_DATA_WIDTH)
    ) u_soc_ctrl (
        .clock                 (clock),
        .ext_reset             (ext_reset),
        .soft_reset            (soft_reset),
        .irq                   (irq),
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
        .s_axi_control_RRESP   (s_axi_control_RRESP),
        .cmd_valid             (core_cmd_valid),
        .cmd_inst              (core_cmd_inst),
        .cmd_ready             (core_cmd_ready),
        .cmd_done              (core_cmd_done),
        .cmd_error             (core_cmd_error),
        .core_busy             (core_status_busy),
        .data_base_addr        (dma_data_base_addr),
        .dma_busy              (dma_busy),
        .axi_rd_err            (dma_axi_rd_err),
        .axi_wr_err            (dma_axi_wr_err)
    );

    // ------------------------------------------------------------------------
    // 2. BNN v2 协议无关计算核心
    // ------------------------------------------------------------------------
    bnn_core_top #(
        .ADDR_WIDTH         (CORE_ADDR_WIDTH),
        .STRIDE_WIDTH       (STRIDE_WIDTH),
        .DMA_DATA_WIDTH     (DMA_DATA_WIDTH),
        .DMA_LEN_WIDTH      (DMA_LEN_WIDTH),
        .IDMA_INT8_NUM      (IDMA_INT8_NUM),
        .WEIGHT_WIDTH       (WEIGHT_WIDTH),
        .ODMA_FP32_NUM      (ODMA_FP32_NUM),
        .MAX_ROWS           (MAX_ROWS),
        .WSRAM_ADDR_WIDTH   (WSRAM_ADDR_WIDTH),
        .WSRAM_READ_LATENCY (WSRAM_READ_LATENCY),
        .WEIGHT_BASE_ADDR   (WEIGHT_BASE_ADDR),
        .MAX_WEIGHT_GROUPS  (MAX_WEIGHT_GROUPS),
        .OSRAM_READ_LATENCY (OSRAM_READ_LATENCY)
    ) u_core (
        .clock              (clock),
        .reset              (core_reset),
        .cmd_valid          (core_cmd_valid),
        .cmd_inst           (core_cmd_inst),
        .cmd_ready          (core_cmd_ready),
        .cmd_done           (core_cmd_done),
        .cmd_error          (core_cmd_error),
        .status_busy        (core_status_busy),
        .mem_rd_cmd_valid   (core_mem_rd_cmd_valid),
        .mem_rd_cmd_ready   (core_mem_rd_cmd_ready),
        .mem_rd_cmd_addr    (core_mem_rd_cmd_addr),
        .mem_rd_cmd_len     (core_mem_rd_cmd_len),
        .mem_rd_data_valid  (core_mem_rd_data_valid),
        .mem_rd_data_ready  (core_mem_rd_data_ready),
        .mem_rd_data        (core_mem_rd_data),
        .mem_rd_data_last   (core_mem_rd_data_last),
        .mem_wr_cmd_valid   (core_mem_wr_cmd_valid),
        .mem_wr_cmd_ready   (core_mem_wr_cmd_ready),
        .mem_wr_cmd_addr    (core_mem_wr_cmd_addr),
        .mem_wr_cmd_len     (core_mem_wr_cmd_len),
        .mem_wr_data_valid  (core_mem_wr_data_valid),
        .mem_wr_data_ready  (core_mem_wr_data_ready),
        .mem_wr_data        (core_mem_wr_data),
        .mem_wr_data_strb   (core_mem_wr_data_strb),
        .mem_wr_data_last   (core_mem_wr_data_last),
        .mem_wr_done        (core_mem_wr_done)
    );

    // ------------------------------------------------------------------------
    // 3. AXI4 Full Master 访存适配引擎
    // ------------------------------------------------------------------------
    bnn_soc_dma #(
        .CORE_ADDR_WIDTH          (CORE_ADDR_WIDTH),
        .DMA_DATA_WIDTH           (DMA_DATA_WIDTH),
        .DMA_LEN_WIDTH            (DMA_LEN_WIDTH),
        .C_M_AXI_ID_WIDTH         (C_M_AXI_GMEM_ID_WIDTH),
        .C_M_AXI_ADDR_WIDTH       (C_M_AXI_GMEM_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH       (C_M_AXI_GMEM_DATA_WIDTH),
        .C_M_AXI_AWUSER_WIDTH     (C_M_AXI_GMEM_AWUSER_WIDTH),
        .C_M_AXI_ARUSER_WIDTH     (C_M_AXI_GMEM_ARUSER_WIDTH),
        .C_M_AXI_WUSER_WIDTH      (C_M_AXI_GMEM_WUSER_WIDTH),
        .C_M_AXI_RUSER_WIDTH      (C_M_AXI_GMEM_RUSER_WIDTH),
        .C_M_AXI_BUSER_WIDTH      (C_M_AXI_GMEM_BUSER_WIDTH),
        .AXI_COHERENT             (AXI_COHERENT),
        .AXI_MAX_BURST_LEN        (AXI_MAX_BURST_LEN)
    ) u_soc_dma (
        .clock              (clock),
        .reset              (core_reset),
        .data_base_addr     (dma_data_base_addr),
        .dma_busy           (dma_busy),
        .axi_rd_err         (dma_axi_rd_err),
        .axi_wr_err         (dma_axi_wr_err),
        .mem_rd_cmd_valid   (core_mem_rd_cmd_valid),
        .mem_rd_cmd_ready   (core_mem_rd_cmd_ready),
        .mem_rd_cmd_addr    (core_mem_rd_cmd_addr),
        .mem_rd_cmd_len     (core_mem_rd_cmd_len),
        .mem_rd_data_valid  (core_mem_rd_data_valid),
        .mem_rd_data_ready  (core_mem_rd_data_ready),
        .mem_rd_data        (core_mem_rd_data),
        .mem_rd_data_last   (core_mem_rd_data_last),
        .mem_wr_cmd_valid   (core_mem_wr_cmd_valid),
        .mem_wr_cmd_ready   (core_mem_wr_cmd_ready),
        .mem_wr_cmd_addr    (core_mem_wr_cmd_addr),
        .mem_wr_cmd_len     (core_mem_wr_cmd_len),
        .mem_wr_data_valid  (core_mem_wr_data_valid),
        .mem_wr_data_ready  (core_mem_wr_data_ready),
        .mem_wr_data        (core_mem_wr_data),
        .mem_wr_data_strb   (core_mem_wr_data_strb),
        .mem_wr_data_last   (core_mem_wr_data_last),
        .mem_wr_done        (core_mem_wr_done),
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
        .m_axi_gmem_RUSER   (m_axi_gmem_RUSER)
    );

endmodule

`default_nettype wire
