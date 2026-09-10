# LY Power Control

**LYHrmer 维护的电机建模、底盘功率分配与仿真验证工程。**

本项目在原仓库的电机功率建模工作上继续开发，参考香港科技大学 ENTERPRIZE RM2024 功率控制资料，补齐独立 C99 控制库、能量预算、Python 参数拟合、动态仿真和自动测试。默认示例为四电机底盘，算法支持 1 至 8 个电机。

这是可运行的工程基础版本。示例参数和仿真工况不是实车标定结果，未宣称比赛通过或绝不超功率。平衡车和轮腿的姿态稳定约束、舵轮转向与驱动功率优先级，需要应用层另行设计。

## 从这里开始

需要 C99 编译器、CMake 3.16+；拟合与拟合测试使用 Python 3 标准库。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/power_control_demo
```

动态仿真直接调用同一个 C 控制库，比较启用功率控制和仅限电流的两套电机系统：

```bash
mkdir -p artifacts/simulation
./build/power_control_sim artifacts/simulation/trace.csv
python3 -s simulations/plot_results.py artifacts/simulation/trace.csv --output-dir artifacts/simulation
```

绘图需要 Matplotlib；运行 C 仿真与测试不需要它。这里的 `-s` 使用系统 Python 包，避开本机用户目录的 NumPy/Matplotlib 版本冲突；在自己的虚拟环境中可省略。安装方法、输出说明和模型假设见 [仿真说明](simulations/README.md)。

![合成动态仿真：功率、速度与储能对比](docs/assets/power_control_sim.png)

这组仿真中，电池侧累计超限能量由 846.589 J 降至 2.233 J，同时速度跟踪 RMSE 从 57.690 rpm 增至 117.011 rpm。仍有模型低估造成的瞬时真实超限，完整结果见 [验证记录](docs/validation.md)。

## 本版本增加了什么

| 模块 | 内容 | 解决的问题 |
| --- | --- | --- |
| C99 核心 | 固定容量、无动态内存、无 RTOS/CMSIS 依赖 | 从特定 STM32 工程中解耦 |
| 电机模型 | 有符号机械功率、一次/二次转速损耗、铜损、空载项 | 明确单位，兼容两类历史模型 |
| 功率分配 | 先限流、再共同缩放；处理制动非单调曲线 | 保留指令比例和符号，显式报告不可行 |
| 能量预算 | NORMAL / CHARGE / BOOST、缓冲降额、电容储能约束、失联降级 | 将策略与电机求解分开 |
| 参数拟合 | 非负最小二乘、独立文件验证、时间错位剔除、JSON/C 头文件 | 不依赖 MATLAB；记录数据和电压假设 |
| 动态仿真 | 转动惯量、载荷、参数偏差、能量积分、通信故障 | 分开验证预测约束与实际功率偏差 |
| 回归与 CI | 数值边界、随机场景、拟合恢复、动态轨迹、GCC/Clang | 让修改有可重复的检查入口 |

控制库采用以下单电机模型，`u` 为原始电流指令，`n` 为同一轴约定下的 rpm：

```text
P(u,n) = k_mech*u*n + k_speed_abs*abs(n)
       + k_speed_sq*n*n + k_current_sq*u*u + p_idle
```

预算约束采用各轮正功率之和 `sum(max(P_i, 0))`，不把回馈功率作为其他电机的额外额度。控制器寻找保持各轮比例的最大可行缩放系数；若在这条指令方向上不存在可行解，则返回 `INFEASIBLE`，应用层必须处理。完整推导及其适用边界见 [模型与分配](docs/model-and-allocation.md)。

## 项目结构

```text
include/ly_power/power_control.h   公共 C API
src/power_control.c               功率分配与能量预算
examples/power_control_demo.c     无硬件依赖的调用示例
tools/fit_power_model.py          离线拟合、验证与参数导出
simulations/                      动态仿真、绘图和假设说明
tests/                            C 数值回归与 Python 拟合测试
docs/                             原理、接入、标定与验证记录
model_fit/                        原仓库 MATLAB 脚本与历史数据
chassis_power_control.c            保留的旧 STM32 接入代码，附边界修复
license.txt                       MIT 许可及原作者、新增工作署名
THIRD_PARTY_NOTICES.md             来源与参考边界
```

旧版 `chassis_power_control.c` 依赖外部板级头文件，不参与新 CMake 构建；新项目从公共 C API 接入。保留原 MATLAB、数据集、报告和 [旧说明](docs/legacy-readme.md)，便于核对历史实现。

## 文档与使用路线

1. 阅读 [模型与分配](docs/model-and-allocation.md)，确定转速、电流与功率测量边界。
2. 按 [参数标定](docs/calibration.md) 采集自己的训练和验证数据，评估误差后导出参数。
3. 运行 [动态仿真](simulations/README.md)，观察限功率后的速度代价、超调与降级行为。
4. 按 [嵌入式接入](docs/integration.md) 接入 PID、裁判系统和超级电容。
5. 查看 [验证记录](docs/validation.md)，区分已经通过的软件检查和待完成的实车验证。

## 来源与维护

本仓库由 **LYHrmer** 维护。原始代码、MATLAB、数据和配套材料保留 Wenhao Lin 的作者与 MIT 许可记录；新增控制库、拟合工具、仿真和工程文档采用同一 MIT 许可。

参考资料：[ENTERPRIZE RM2024-PowerModule](https://github.com/hkustenterprize/RM2024-PowerModule)、[RoboMaster 论坛原文](https://bbs.robomaster.com/article/54121)、用户提供的 2024-09-22 压缩包。新增实现借鉴建模和能量管理思路，未导入其队内控制代码、矩阵库、图片和视频。详细来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

后续重点是使用本机数据完成参数标定、控制周期耗时测量和闭环负载试验，再考虑具有数据有效性门控的在线 RLS，以及舵轮/平衡底盘专用分配策略。贡献约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。
