# 嵌入式接入

新版入口为 `include/ly_power/power_control.h` 和 `src/power_control.c`。将这两个文件加入工程，添加 `include/` 搜索路径，启用 C99，并链接编译器的数学库。无需 CMSIS、FreeRTOS、动态内存或特定裁判协议。桌面 CMake 工程用于验证算法，不是可直接烧录的 STM32 固件。

## 控制周期中的位置

```text
有效的电机反馈 -> 速度 PID -> 未分配电流指令
裁判/缓冲/电容状态 -> lp_compute_budget -> 功率预算
模型 + rpm + 指令 + 预算 -> lp_allocate -> 分配后电流
状态处理 + PID 抗饱和 + 整数量化检查 -> CAN 发送
```

`lp_allocate()` 支持 1 至 8 个电机，不负责底盘运动学解算。麦轮或全向轮的电机方向转换应在传入前完成，模型中的正转速和正电流必须对应同一机械方向。`current_limit` 使用驱动器原始指令单位，不能把安培数直接放入。

下面展示单周期调用骨架，`models` 必须来自针对本机的有效标定，`rpm` 和 `pid_request` 应为同一控制时刻的数组：

```c
#include "ly_power/power_control.h"

/* These values illustrate policy, not competition rules or calibration. */
static const lp_budget_config_t policy = {
    .fallback_limit_w = 40.0f,
    .margin_w = 5.0f,
    .charge_reserve_w = 15.0f,
    .buffer_target_j = 40.0f,
    .buffer_gain_w_per_j = 1.0f,
    .cap_reserve_j = 30.0f,
    .boost_power_w = 50.0f,
    .boost_horizon_s = 2.0f,
    .absolute_max_w = 150.0f
};

uint32_t allocate_tick(const lp_motor_model_t models[4],
                       const float rpm[4], const float pid_request[4],
                       const lp_budget_input_t *telemetry,
                       lp_allocation_t *allocated)
{
    lp_budget_t budget;
    uint32_t flags = lp_compute_budget(&policy, telemetry, &budget);
    /* Returning an error requires the caller to enter its defined fault state. */
    if (flags & (LP_STATUS_INVALID_ARGUMENT | LP_STATUS_NUMERIC_ERROR)) {
        *allocated = (lp_allocation_t){0};
        allocated->flags = flags;
        return flags;
    }
    flags |= lp_allocate(models, rpm, pid_request, 4,
                         budget.budget_w, allocated);
    return flags;
}
```

此骨架假设输出指针有效。返回值合并预算和分配状态，不能只读取 `allocated->flags` 判断裁判或电容是否过期。完整可编译的基本调用见 `examples/power_control_demo.c`。

## 必须处理的状态

| 状态 | 含义 | 应用层动作 |
| --- | --- | --- |
| `LP_STATUS_CURRENT_CLAMPED` | 某轮已触及电流限幅 | 记录饱和，执行抗积分饱和 |
| `LP_STATUS_POWER_LIMITED` | 缩放因功率限制而生效 | 使用分配后的实际电流更新 PID 抗饱和 |
| `LP_STATUS_INFEASIBLE` | 在当前共同缩放方向上无可行命令 | 执行经实机验证的故障/制动策略，不能视为功率达标 |
| `LP_STATUS_INVALID_ARGUMENT` / `INVALID_MODEL` / `NUMERIC_ERROR` | 输入、参数或数值范围失效 | 禁止直接发送结果，进入本机故障处理 |
| `LP_STATUS_JUDGE_STALE` / `BUFFER_STALE` / `CAP_STALE` | 反馈不再可信 | 记录告警并遵守预算降级结果 |
| `LP_STATUS_MODE_DEGRADED` | BOOST 因反馈问题退化 | 操作端显示受限状态，等待新鲜有效反馈 |
| `LP_STATUS_NEGATIVE_BUDGET` | 负预算按零处理 | 检查上层策略和单位 |

无可行解时，库返回当前缩放方向上预测功率最低的指令。这可能仍包含制动力矩，不能把它等同于停机命令。车体安全动作属于应用层，平衡底盘尤其不能忽略姿态稳定约束。

## 裁判、电容与反馈时效

`judge_valid`、`buffer_valid`、`cap_valid` 表示采样新鲜、数值可信且属于本机。必须根据数据时间戳设置，不能永久写为 `1`。`boost_horizon_s` 至少覆盖下一次可信能量更新之前的时间；不断复用旧能量会破坏能量预算。

`cap_energy_j` 为高于硬件最低工作电压的可用能量，例如 `0.5*C*(V*V-V_min*V_min)`。`cap_reserve_j` 在此基础上再保留一部分。不要直接把百分数或旧工程的 `0..255` 数字当作焦耳。

预算是电机侧允许的功率。BOOST 时它可以高于裁判侧上限，电容控制器仍须独立限制电池侧输入；不能把 `budget_w` 直接发送为电容充电输入限额。电容失效时，取消额外预算并按照本机电源拓扑处理旁路/断路状态。

裁判上限和协议由工程提供，库不内置赛季等级表。默认 fallback、储能、功率和响应周期均是配置示例，应根据实际供电和赛规确定。

## 电流量化与 PID 抗饱和

库输出浮点电流，不执行 CAN 整数编码。即使简单向零取整，在制动区也可能增加模型预测的正功率。因此应以实际发送的整数命令再次检查模型消耗，并设置经过验证的量化与模型误差裕量。不能把连续浮点解的边界保证直接扩展成硬件功率保证。

发生限流或限功率时，冻结会继续加剧饱和的积分，或使用分配前后电流差做反算抗饱和。反馈控制器若持续累积未能执行的电流，会在限额恢复时产生冲击。

## 控制周期与实机验证

核心使用 `double` 中间计算处理极端参数和制动边界。STM32F4 通常没有双精度硬件加速，必须测量目标编译选项下的最坏执行时间，确认它适合所需控制周期；主机仿真运行速度不能替代 MCU 耗时测试。

优先在台架完成参数与功率测量对比，再检查低电容能量、坡道/堵转、反向制动、无线/裁判丢帧与传感器延迟。仿真只覆盖所建模型，其测得的超调和跟踪代价是选择裕量与下一步实验的依据。
