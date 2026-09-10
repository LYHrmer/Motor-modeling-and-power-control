# 功率控制动态仿真

这里提供一个直接调用新版 C99 功率核心的确定性动态仿真，以及 CSV 绘图与统计脚本。仿真用来验证数值约束、反馈降级和闭环行为，并展示功率限制带来的速度代价。

所有电机、供电和控制参数都是合成参数。仿真不是 LYHrmer 的实车采集数据，也不能替代硬件标定、底盘测试或赛场功率验证。

## 快速运行

从仓库根目录运行，不需要嵌入式工具链或第三方 C 库：

```sh
mkdir -p build/simulation
cc -std=c99 -O2 -Wall -Wextra -Wpedantic -Iinclude \
  src/power_control.c simulations/power_control_sim.c -lm \
  -o build/simulation/power_control_sim
build/simulation/power_control_sim --check-only
build/simulation/power_control_sim build/simulation/results.csv
python3 simulations/plot_results.py build/simulation/results.csv \
  --output-dir build/simulation/report
```

项目构建系统也可集成 `power_control_sim` 可执行文件和 `--check-only` 回归。直接编译命令给出了最小依赖，不要求先安装整个项目。

绘图需要 Matplotlib，读取和统计只使用 Python 标准库，不需要 pandas。若机器的用户级 NumPy 与系统 Matplotlib 有二进制版本冲突，而系统包本身兼容，可将绘图命令中的 `python3` 改为 `python3 -s`，禁用用户级 site packages；这不会修改已安装环境。

也可以使用可选的隔离环境安装绘图依赖：

```sh
python3 -m venv .venv-sim
.venv-sim/bin/python -m pip install matplotlib
.venv-sim/bin/python simulations/plot_results.py build/simulation/results.csv \
  --output-dir build/simulation/report
```

上述安装需要网络，只在当前环境缺少可用 Matplotlib 时执行；本次运行使用现有系统包，没有安装或修改依赖。

无参数运行 C 程序会在当前目录生成 `power_control_sim.csv`，一个路径参数指定 CSV 位置，`--check-only` 只进行数值回归和终端统计。

绘图输出：

- `power_control_sim.png`：六面板功率、速度、能量和缩放曲线。
- `power_control_sim.svg`：相同曲线的矢量版本。
- `summary.md`：两条轨迹的误差、功率超调、能量统计和解释。

## 两套独立的被控对象

仿真同时运行 `controlled` 和 `unlimited`。它们接受相同目标和外部负载，但各自拥有电机转速、PI 积分、电容能量和缓冲能量状态。

`controlled` 每周期调用 `lp_compute_budget` 和 `lp_allocate`；`unlimited` 使用相同 PI 与电流限制，将分配器预算设为足够大的数以保留仅限流行为。基线也计算预算供比较，但不执行这个功率约束。由于能量状态不同，两条轨迹记录的预算不一定相同。

每个电机具有不同的转矩常数、转动惯量、阻尼、电阻、速度损耗、静态损耗和电流上限。四个目标速度分别乘以 `0.95, 1.05, 1.02, 0.98`，模拟轻微轮间差异。

模型采用同一轴的原始电流指令和转速：

\[
I=u\frac{20}{16384},\qquad
\tau=K_t I,\qquad
J\dot\omega=\tau-b\omega-\tau_{\mathrm{load}},\qquad
\mathrm{rpm}=\omega\frac{60}{2\pi}.
\]

外部阻力为指定负载幅值乘 `tanh(omega / 0.5)`，平滑地反向并避免零速处的符号跳变。积分采用显式 Euler，步长 `0.002 s`，控制和动力学均为 `500 Hz`，总时长 `28 s`。

实际电机侧输入功率根据物理参数计算：

\[
P_{\mathrm{plant}}=\sum_i\max\left(
\tau_i\omega_i+R_iI_i^2+
c_{1,i}|\mathrm{rpm}_i|+c_{2,i}\mathrm{rpm}_i^2+p_{0,i},0\right).
\]

控制器系数由这些物理参数换算后，分别乘 `0.89, 0.90, 0.91, 0.92`，故意形成约 8% 至 11% 的低估。控制器不直接访问 plant 的真实系数来修正当前输出。

这比把限制后的命令重新代入同一个静态公式更进一步：输出电流会改变下一周期的转速、速度误差、电容和缓冲状态。但它仍共享相近的功率模型结构，未覆盖磁饱和、PWM、电调电流环延迟、机械间隙、轮胎滑移、接触动力学及测量噪声。

速度控制使用 PI，比例增益 `100 raw/rpm`，积分增益 `120 raw/(rpm*s)`。输出分配后采用增益 `12 /s` 的回算抑制积分饱和，并约束积分范围；基线也采用相同策略。

## 工况

| 时间 / s | CSV 场景 | 输入变化与目的 |
| --- | --- | --- |
| 0 至 2 | `startup` | 前 0.5 s 静止，再请求 180 rpm，观察启动和限流 |
| 2 至 5 | `cruise` | 260 rpm，观察轻载稳定跟踪 |
| 5 至 8 | `load_step` | 外部阻力幅值由 0.12 增至 0.65 N m，各轮另有小幅差异 |
| 8 至 11 | `judge_step` | 合成功率阈值由 90 W 降至 45 W |
| 11 至 14 | `reverse` | 目标变为 -220 rpm，阈值为 75 W，覆盖制动与反向 |
| 14 至 16 | `charge` | 目标 120 rpm，进入 CHARGE 并预留 15 W |
| 16 至 19 | `boost` | 目标 380 rpm，进入 BOOST，使用有限储能 |
| 19 至 21 | `cap_reserve` | 持续高请求，观察储能接近预留量后的额度收缩 |
| 21 至 23 | `cap_stale` | 目标 220 rpm，负载降至 0.28 N m，冻结电容反馈 |
| 23 至 25 | `referee_stale` | 目标 180 rpm，恢复电容反馈，冻结裁判上限反馈 |
| 25 至 27 | `buffer_stale` | 目标 240 rpm，恢复裁判反馈，冻结缓冲反馈 |
| 27 至 28 | `recovery` | 恢复反馈，NORMAL，目标 160 rpm |

后三个反馈故障段仍请求 BOOST，以确认不可信反馈会报告降级并撤销增强额度。反馈正常时每 `20 ms` 更新一次，超过 `100 ms` 未更新才判陈旧，因此故障段开头存在明确的超时检测延迟。

`cap_reserve` 不是在某个时刻强行改写电容能量；它延续动力学和能量积分。储能接近预留量的具体时间依赖负载、参数与前面场景，曲线如实展示该结果。

## 供电与能量边界

电容可用能量范围为 `0..90 J`，初值 `70 J`；额外保护预留为 `12 J`，BOOST 时域为 `0.6 s`。最大额外预算和模拟硬件最大放电功率均为 `65 W`。这些都是示例设置，不是任何当前赛季规则或通用硬件规格。

电容位于电池与电机之间，模拟一个以当前供电阈值为目标的独立电容控制器：

- 电机消耗高于供电阈值时，电容补偿差额，但受最大放电功率和剩余能量约束。
- 电机消耗低于供电阈值时，用余量充电，最大充电功率 `15 W`，受剩余容量约束。
- 电容无法补足差额时，电池侧实际功率可以超过阈值，缓冲能量随之下降。
- 电容反馈陈旧表示通信故障，物理电容仍按上述自主供电规则运行。

理想转换效率设为 1，不计回馈，能量积分为：

\[
P_{\mathrm{source}}=P_{\mathrm{plant}}+P_{\mathrm{charge}}-P_{\mathrm{discharge}},
\quad
E_{\mathrm{cap}}(t+\Delta t)=
E_{\mathrm{cap}}+(P_{\mathrm{charge}}-P_{\mathrm{discharge}})\Delta t.
\]

合成裁判缓冲上限为 `40 J`，初值 `35 J`，按下式积分并截断到合法范围：

\[
E_{\mathrm{buffer}}(t+\Delta t)=
\operatorname{clip}\left(E_{\mathrm{buffer}}+
(P_{\mathrm{ref}}-P_{\mathrm{source}})\Delta t,0,40\right).
\]

缓冲为零时继续积分并记录超调，不模拟扣血、裁判系统内部滤波或比赛判罚。电池侧超阈值积分与电机侧超预算积分分别统计，不能混用。

## 自动回归检查

`--check-only` 与生成 CSV 的路径执行同样的闭环仿真。失败返回非零退出码，断言不受 `NDEBUG` 影响。

- 动态状态、输出、功率和能量始终有限。
- 输出电流不超过电机上限，不放大输入请求、不反转其符号。
- 除显式 `INFEASIBLE` 状态外，控制器输出的预测功率不超过预算及数值容差。
- 电容与缓冲能量保持在定义域内。
- 实际覆盖电流限幅、功率限幅和三路反馈陈旧及模式降级 flags。

物理 plant 的实际功率不设人为“必须通过”的门槛。故意保留模型误差后，真实超调由报告量化；程序通过表示这些指定的数值不变量通过，不表示真实功率在所有工作点都满足预算。

## CSV 与结果解释

每个 `2 ms` 控制周期记录两行，分别对应两套独立轨迹。转速、电容和缓冲字段是周期开始时的状态；功率与输出是本周期计算结果，能量随后按本周期功率积分。

主要列为 `time_s`、`scenario`、`run`、`mode`、`budget_w`、`model_power_w`、`plant_power_w`、`source_power_w`、`referee_limit_w`、`cap_energy_j`、`buffer_energy_j`、`mean_rpm`、`target_rpm`、`tracking_error_rpm`、`scale`、两组 flags 和最大输出原始电流。

`tracking_error_rpm` 是当周期四电机误差的均方根；整段 RMSE 再对其平方按时间积分。总消耗能量差也包含速度跟踪差异造成的机械做功差异，不直接代表效率提升。

查看结果时同时关注源侧超调、能量恢复与速度损失。向实车推广前仍需测量真实参数、通信延迟与控制周期，并针对目标驱动器和电容供电边界重新验证。
