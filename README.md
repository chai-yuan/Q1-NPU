# Q1-NPU

**面向 GGUF `Q1_0`（1.125-bit 分组缩放 1-bit 权重）的专用神经网络推理加速器与全栈软硬件原型。**

Q1-NPU 从量化算术契约、可综合 SystemVerilog RTL、Verilator 协同仿真、55nm 综合/STA 评估到零第三方依赖的 C11 推理引擎构建了完整闭环，当前以 **1-bit 大语言模型（如 `Qwen3-Bonsai-1.7B-Q1_0`）** 推理为首要验证目标，并面向通用推理场景持续演进。

> **命名说明**：本仓库原名为 `BNN-train-infer`，现已更名为 **Q1-NPU**。为保持开发连续性，内部目录、构建目标与 C/RTL 符号目前暂保留历史 `bnn-*` / `BNN_*` 命名，将在后续架构迭代中逐步统一迁移。

---

## 为什么 Q1-NPU 不是传统 BNN？

传统意义上的 BNN（Binary Neural Network）通常指早期 **W1A1**（权重与激活均为 1-bit、基于 XNOR + Popcount）的网络，主要停留在 MNIST/CIFAR 等小规模视觉模型。Q1-NPU 面向的是新一代**分组缩放 1-bit 权重（`Q1_0` 家族）**推理硬件：

| 维度 | 传统 BNN（W1A1） | Q1-NPU（当前版本） |
| :--- | :--- | :--- |
| 权重格式 | 纯 1-bit 符号（无分组缩放或仅层缩放） | **GGUF `Q1_0`**：每 128 个 ±1 权重共享 1 个 FP16 scale（18 B/块，1.125 bpw） |
| 激活与计算 | 1-bit 激活，XNOR + Popcount | **W1A8**：INT8 激活（支持片上 FP32→INT8 动态量化）、组内精确整数点积 + 组间 FP32 累加 |
| 目标负载 | 轻量 CV 分类 | 原生运行 **Qwen3-1.7B (`Q1_0` GGUF)** 等 1-bit LLM，兼顾通用线性层/MLP 推理 |
| 工程形态 | 常见为单模块算法仿真或教学实验 | 完整 64-bit ISA、双 1RW SRAM、DMA 突发读写、TCP 远程 RTL 协仿、55nm 综合与 OpenSTA 基线 |

---

## 核心特性与当前进展

- **可综合计算核心（[`bnn-hw-v2/bnn_core/`](bnn-hw-v2/bnn_core/)）**
  - 64-bit 串行单阶段原子指令集（`SET_SRC_STRIDE` / `SET_DST_STRIDE` / `LOAD_WEIGHT` / `COMPUTE` / `WRITEBACK`），支持 2D 步长访存与尾列字节写掩码；
  - 片上双单端口 `1RW` SRAM 浅包装（Weight SRAM `128×144b`、Result SRAM `4096×128b`），预留厂商 SRAM 宏替换边界；
  - 无外部浮点 IP 依赖：内置可综合 `bnn_fp_math`（FP16→FP32、FTZ/RTZ 乘加与片上行动态 INT8 量化）。
- **端到端验证与协同仿真（[`bnn-hw-v2/simple_test/`](bnn-hw-v2/simple_test/) & [`bnn-hw-v2/remote_sim/`](bnn-hw-v2/remote_sim/)）**
  - 离线 Verilator + DPI-C 回归：基于真实 Qwen3 权重验证 `GEMM`（跨 K 块累加）与 `GEMM_F32`，实测最大相对误差 `1.45e-7`；
  - TCP 远程 RTL 协同仿真：通过 `ACCE=remote` 将 C11 推理引擎直连 Verilator 仿真进程，内置周期级性能探针与 FSM 状态直方图。
- **55nm 综合与时序/面积基线（[`bnn-hw-v2/sta/`](bnn-hw-v2/sta/)）**
  - 基于 `sv2v` → Yosys → OpenSTA（ICS55 `H7CR` RVT 标准单元，post-synthesis 理想时钟，SRAM 为 `TARGET_ASIC` 占位模型）：
    - 典型 corner (`typ_tt_1p2_25`)：**230.4 MHz**
    - 最差 corner (`ss_cworst_1p08_m40`)：**122.4 MHz**
    - 纯逻辑面积：**227,054 µm²**（约 0.227 mm²，96,370 个标准单元，12,959 个 DFF）
- **C11 推理引擎与后端抽象（[`bnn-sw-v2/`](bnn-sw-v2/) & [`bnn-accelerator/`](bnn-accelerator/)）**
  - 统一 Accelerator API v2，支持 `ACCE=soft`（C 软件参考）与 `ACCE=remote`（RTL 协同仿真）无缝切换；
  - 完整打通两条推理闭环：`llm`（Qwen3 GGUF `Q1_0` 文本生成）与 `classify`（`bnn-mlp` 分类）。
- **调试与分析工具链（[`tools/`](tools/)）**
  - 独立 NumPy 参考与逐位置 logits/KL/PPL 精度比对、纯前端 GGUF 结构检视器 `gguf-lens` 与 RTL 波形辅助工具 `wave-deck`。

---

## 目录结构

```text
bnn-hw-v2/             Q1-NPU 硬件部分：可综合 RTL (bnn_core/)、Verilator 离线回归 (simple_test/)、
                       TCP 协同仿真 (remote_sim/)、Yosys+OpenSTA 流程 (sta/)
bnn-sw-v2/             C11 推理引擎：公共 API、GGUF 加载、BPE Tokenizer、Qwen3 与 MLP 模型实现
bnn-accelerator/       Accelerator API v2 接口定义、soft 参考后端与 remote RTL 后端
bnn-infer-v2.c         推理命令行入口（llm / classify）
tests/accelerator/     加速器 API 与软件参考数值边界回归测试
tools/                 Qwen3 NumPy 参考、精度分析 (precision/)、gguf-lens、wave-deck、Unicode 表生成
train/                 训练与模型导出脚本（Qwen3 / MNIST-MLP）
references/            参考开源加速器项目（Git Submodules：`tvm-vta`、`redmule`、`gemmini`）
```

---

## 快速开始

### 1. 构建推理引擎与 CLI

依赖 C11 编译器、Make、`ar` 与 POSIX 单调时钟；数值编译选项固定为 `-fno-fast-math -ffp-contract=off`。

```bash
# 方式一：C 软件参考后端（默认）
make ACCE=soft

# 方式二：连接 bnn-hw-v2/remote_sim 的 RTL 协同仿真后端
make ACCE=remote
```

产物为根目录 `bnn-infer-v2` 与 `build/<ACCE>/libbnn-engine.a`。

### 2. 运行 Qwen3 (`Q1_0` GGUF) 推理

```bash
# 原始文本续写
./bnn-infer-v2 llm Qwen3-Bonsai-1.7B-Q1_0.gguf \
    --prompt "The capital of France is" --steps 4

# Qwen3 单轮对话模板
./bnn-infer-v2 llm Qwen3-Bonsai-1.7B-Q1_0.gguf \
    --chat --prompt "你好" --steps 40

# 按 token ID 文件输入（空白分隔十进制整数）
./bnn-infer-v2 llm Qwen3-Bonsai-1.7B-Q1_0.gguf --ids ids.txt --ctx 1024
```

常用参数：`--prompt TEXT` / `--ids FILE`（默认提示词 `春天`）、`--steps N`（默认 40）、`--ctx N`（默认 1024，对应 KV Cache 约 235 MB）、`--chat`（启用单用户模板）。当前采用贪心解码，遇 `<|im_end|>` / `<|endoftext|>` 停止。

### 3. 运行 RTL 协同仿真推理（端到端）

```bash
# 终端 1：启动 Verilator TCP 仿真服务器（默认监听 127.0.0.1:12345）
make -C bnn-hw-v2/remote_sim run

# 终端 2：编译 remote 后端并驱动 RTL 执行推理
make ACCE=remote
./bnn-infer-v2 llm qwen3-zh-5k.gguf --ids ids.txt --steps 1 --ctx 8
```

支持通过环境变量 `BNN_REMOTE_ADDR` 与 `BNN_REMOTE_MEM_BYTES`（默认 `512 MiB`）配置连接地址与设备内存容量。

### 4. 运行分类闭环（`bnn-mlp`）

```bash
# 训练并导出 Q1_0 GGUF 模型与测试集（需要 Python 3、NumPy、CPU 版 PyTorch）
python3 train/MNIST-MLP/train_mnist_mlp.py

# 执行推理与精度评估（当前默认配置导出模型测试集准确率 ~98.3%）
./bnn-infer-v2 classify train/MNIST-MLP/out/mnist_mlp.gguf \
    --input train/MNIST-MLP/out/test-inputs.f32 \
    --labels train/MNIST-MLP/out/test-labels.u8 --batch 64
```

---

## 测试与硬件分析

```bash
# 1. Accelerator API v2 与软件参考边界测试（不依赖 Verilator）
make -C tests/accelerator test

# 2. RTL 离线固定用例回归（Qwen3 真实权重，需 Verilator + C++ 工具链）
make -C bnn-hw-v2/simple_test test

# 3. TCP 远程协同仿真协议与端到端校验
make -C bnn-hw-v2/remote_sim test

# 4. 55nm 综合与 OpenSTA 时序/面积报告（需 sv2v、Yosys、OpenSTA 与 PDK）
make -C bnn-hw-v2/sta
```

---

## 架构与文档导航

- **硬件微架构、ISA、算术契约与仿真协议**：详见 [`bnn-hw-v2/README.md`](bnn-hw-v2/README.md)
- **Accelerator API v2、内存视图、量化语义与 GGUF 布局取舍**：详见 [`bnn-accelerator/README.md`](bnn-accelerator/README.md)
- **55nm 综合流程、Corner 选择与 STA 限制说明**：详见 [`bnn-hw-v2/sta/README.md`](bnn-hw-v2/sta/README.md)
- **精度对比与检视工具**：[`tools/precision/README.md`](tools/precision/README.md)、[`tools/gguf-lens/README.md`](tools/gguf-lens/README.md)、[`tools/wave-deck/README.md`](tools/wave-deck/README.md)、[`tools/arch-atlas/README.md`](tools/arch-atlas/README.md)

---

## 演进路线（Roadmap）

以下方向面向后续版本迭代规划，当前版本仍严格锁定 `Q1_0`（`g=128`）+ W1A8 基线：

1. **微架构吞吐与物理实现**：将串行点积单元升级为平衡并行加法树，引入权重双组乒乓预取与拍宽并行解包；完善 `bnn_top.sv` 外层 AXI4 Master 总线接口，并替换真实 SRAM 宏推进布局布线后评估。
2. **量化格式与精度扩展**：以 `Q1_0` 为核心，计划扩展更细粒度分组缩放变体（如 `g=64`、每 64 权重共享 1 个 FP16 scale 的 1.25-bit 格式）；对外数据接口计划从 FP32 演进为 **FP16 / BF16 / INT**，并探索 W1A12 等更高激活位宽模式。
3. **生态与工作负载拓展**：在持续深耕 1-bit LLM 推理的同时，扩展面向更多网络结构与通用推理场景的算子与软件栈支持，并在后续大版本中将内部 `bnn-*` / `BNN_*` 命名统一迁移至 `q1-*` / `Q1_*`。
