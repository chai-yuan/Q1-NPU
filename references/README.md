# 参考项目 (`references/`)

本目录以 Git Submodule 方式收录与 **Q1-NPU** 定位相近或可借鉴的开源神经网络加速器项目，用于微架构设计参考、软件栈/驱动对照以及后续性能与面积对比基准。

> 克隆本仓库时若未拉取子模块，可执行：
> ```bash
> git submodule update --init --recursive
> ```

## 收录清单

| 目录 | 上游仓库 | 定位与特点 | 对 Q1-NPU 的核心参考价值 |
| :--- | :--- | :--- | :--- |
| [`tvm-vta/`](tvm-vta/) | [apache/tvm-vta](https://github.com/apache/tvm-vta) | Apache TVM 生态的开源通用张量加速器（VTA，默认 INT8×INT8→INT32） | 128-bit 指令集与 Fetch/Load/Compute/Store 解耦流水（FIFO + 信号量依赖同步）、Micro-op 循环加速、Shell/VME/VCR 总线解耦层、Verilator DPI (`TSIM`) 与驱动接口 |
| [`redmule/`](redmule/) | [pulp-platform/redmule](https://github.com/pulp-platform/redmule) | PULP 平台开源混合精度矩阵运算引擎（HWPE 模板，支持 FP16 / FP8 输入与自动 Cast、可参数化 2D CE 阵列，纯 SystemVerilog） | SystemVerilog 可综合微架构组织、输入/输出在线格式转换（`castin`/`castout`）、硬件 Tiler + 轻量状态机调度、X/W/Z 缓冲与流式访存（`streamer`）及双接口封装（MMIO / XIF） |
| [`gemmini/`](gemmini/) | [ucb-bar/gemmini](https://github.com/ucb-bar/gemmini) | UC Berkeley 开源全系统脉动阵列 DNN 加速器生成器（Chipyard / Chisel 生态，支持 WS/OS 脉动阵列与 RoCC 协处理器接口） | 权重驻留/输出驻留脉动阵列（`Mesh`/`PE`/`MeshWithDelays`）、带缩放/激活/归一化的累加器后处理（`AccumulatorScale`/`Normalizer`）、硬件循环展开（`LoopMatmul`/`LoopConv`）与多 Bank Scratchpad 组织 |

## 重点研读入口

### `tvm-vta`
- **ISA 与微操作定义**：[`tvm-vta/include/vta/hw_spec.h`](tvm-vta/include/vta/hw_spec.h)
- **驱动与内存管理接口**：[`tvm-vta/include/vta/driver.h`](tvm-vta/include/vta/driver.h)
- **Chisel RTL 核心（Fetch / Load / Compute / Store / Semaphore）**：[`tvm-vta/hardware/chisel/src/main/scala/core/`](tvm-vta/hardware/chisel/src/main/scala/core/)
- **Shell 与片外总线封装（VCR / VME / AXI / SimShell）**：[`tvm-vta/hardware/chisel/src/main/scala/shell/`](tvm-vta/hardware/chisel/src/main/scala/shell/)
- **Verilator DPI-C 仿真桥接（TSIM）**：[`tvm-vta/hardware/dpi/tsim_device.cc`](tvm-vta/hardware/dpi/tsim_device.cc)、[`tvm-vta/src/tsim/tsim_driver.cc`](tvm-vta/src/tsim/tsim_driver.cc)

### `redmule`
- **顶层与参数包**：[`redmule/rtl/redmule_pkg.sv`](redmule/rtl/redmule_pkg.sv)、[`redmule/rtl/redmule_top.sv`](redmule/rtl/redmule_top.sv)、[`redmule/rtl/redmule_mm_wrap.sv`](redmule/rtl/redmule_mm_wrap.sv)
- **计算阵列与在线类型转换**：[`redmule/rtl/redmule_engine.sv`](redmule/rtl/redmule_engine.sv)、[`redmule/rtl/redmule_ce.sv`](redmule/rtl/redmule_ce.sv)、[`redmule/rtl/redmule_castin.sv`](redmule/rtl/redmule_castin.sv)、[`redmule/rtl/redmule_castout.sv`](redmule/rtl/redmule_castout.sv)
- **控制、Tiler 与流式访存**：[`redmule/rtl/redmule_ctrl.sv`](redmule/rtl/redmule_ctrl.sv)、[`redmule/rtl/redmule_scheduler.sv`](redmule/rtl/redmule_scheduler.sv)、[`redmule/rtl/redmule_tiler.sv`](redmule/rtl/redmule_tiler.sv)、[`redmule/rtl/redmule_streamer.sv`](redmule/rtl/redmule_streamer.sv)
- **片上缓冲与软件驱动**：[`redmule/rtl/x_buffer/`](redmule/rtl/x_buffer/)、[`redmule/rtl/w_buffer/`](redmule/rtl/w_buffer/)、[`redmule/rtl/z_buffer/`](redmule/rtl/z_buffer/)、[`redmule/sw/hal_redmule.h`](redmule/sw/hal_redmule.h)

### `gemmini`
- **ISA 与顶层控制器**：[`gemmini/src/main/scala/gemmini/GemminiISA.scala`](gemmini/src/main/scala/gemmini/GemminiISA.scala)、[`gemmini/src/main/scala/gemmini/Controller.scala`](gemmini/src/main/scala/gemmini/Controller.scala)、[`gemmini/src/main/scala/gemmini/ReservationStation.scala`](gemmini/src/main/scala/gemmini/ReservationStation.scala)
- **脉动阵列与数据流**：[`gemmini/src/main/scala/gemmini/Mesh.scala`](gemmini/src/main/scala/gemmini/Mesh.scala)、[`gemmini/src/main/scala/gemmini/MeshWithDelays.scala`](gemmini/src/main/scala/gemmini/MeshWithDelays.scala)、[`gemmini/src/main/scala/gemmini/PE.scala`](gemmini/src/main/scala/gemmini/PE.scala)
- **片上存储与后处理（Scale / Activation / Norm）**：[`gemmini/src/main/scala/gemmini/Scratchpad.scala`](gemmini/src/main/scala/gemmini/Scratchpad.scala)、[`gemmini/src/main/scala/gemmini/AccumulatorMem.scala`](gemmini/src/main/scala/gemmini/AccumulatorMem.scala)、[`gemmini/src/main/scala/gemmini/AccumulatorScale.scala`](gemmini/src/main/scala/gemmini/AccumulatorScale.scala)、[`gemmini/src/main/scala/gemmini/Normalizer.scala`](gemmini/src/main/scala/gemmini/Normalizer.scala)
- **硬件循环展开与 DMA**：[`gemmini/src/main/scala/gemmini/LoopMatmul.scala`](gemmini/src/main/scala/gemmini/LoopMatmul.scala)、[`gemmini/src/main/scala/gemmini/LoopConv.scala`](gemmini/src/main/scala/gemmini/LoopConv.scala)、[`gemmini/src/main/scala/gemmini/DMA.scala`](gemmini/src/main/scala/gemmini/DMA.scala)
