# 参数标定与数据质量

先定义测量边界，再拟合系数。原始 `model_fit/datasets/` 为仓库历史数据，不能作为 LYHrmer 新采集的实车结果；原 MATLAB 中的 `24.56 V` 也是历史脚本假设，不代表每个 CSV 的实际母线电压。

## 采集自己的数据

工具默认读取逗号分隔的单电机 CSV：

```csv
rpm,current_command,power_w
1200,3000,12.4
```

该行仅演示格式。`current_command` 必须是经过限功率、限流及协议量化后真正发送给电调的指令，不能使用未执行的 PID 原始输出。`power_w` 必须是该电机与相应驱动器的输入功率；不要把四轮总功率与单轮指令拼在一行，也不要把电容充放电时的裁判电池侧功率当作电机侧功率。建议同时保留原始电压、电流、时间戳、反馈电流、温度与实验工况，先做同步处理，再导出本格式。

需要覆盖不同转速、正反转、负载和电流，尤其要避免所有数据都来自单一恒速或几乎相同的负载。训练与验证必须按独立采集文件分开；从同一条轨迹隔行抽样不能替代独立实验。工具会拒绝重名路径重用和明显秩亏，但不可能仅凭 CSV 证明两次实验相互独立。

确认 `k_mech` 的轴约定与电流标度后执行：

```bash
mkdir -p artifacts/calibration
python3 tools/fit_power_model.py training.csv \
  --validation validation.csv \
  --k-mech 1.99688994e-6 --current-limit 16000 \
  --speed-model quadratic \
  --output artifacts/calibration/model.json \
  --header artifacts/calibration/model.h
```

上述机械系数和限流值沿用历史示例，只是演示参数。更换电机、减速比、速度测量轴或驱动器指令标度后必须重算。拟合工具固定机械系数，只拟合非负损耗项，防止用错误的机械项和任意负损耗相互抵消。

`--speed-model quadratic` 拟合二次转速损耗，`linear` 使用绝对转速项，`combined` 同时保留两项。模型越复杂，对数据激励和独立验证要求越高；工具对缩放后的法方程做秩检测，并枚举最多四个损耗项的有效集合求解非负最小二乘。

## 回放原仓库数据

```bash
python3 tools/fit_power_model.py \
  model_fit/datasets/ramp.csv model_fit/datasets/ramp2.csv \
  --validation model_fit/datasets/sin.csv model_fit/datasets/test1.csv \
  --format legacy --bus-voltage 24.56 \
  --k-mech 1.99688994e-6 --current-limit 16000 \
  --output artifacts/calibration/legacy_example.json \
  --header artifacts/calibration/legacy_example.h
```

旧格式为分号分隔，工具使用 `give_current`、`motor_chassis[0].speed_rpm` 和母线 `current`，通过显式电压假设换算功率。默认剔除三路采样时间跨度超过 `5 ms` 的行，可用 `--max-skew-ms` 调整；这只是时间戳筛选，不会补偿传感器滤波延迟或自动重采样。

本次按上述历史假设回放时，非负损耗解全部落到零边界，说明该组数据与所用假设不能直接支撑可部署的损耗参数。工具会在 JSON、终端和导出头文件提示该问题，不能把“拟合程序运行成功”理解为“电机已完成标定”。请先核查供电电压、母线电流标度与零偏、符号、时间对齐和测试边界。具体误差见 [验证记录](validation.md)。

## 如何决定是否采用参数

JSON 保留训练/验证文件、有效行数、错位/无效行数、假设电压、模型系数以及 RMSE、MAE、偏差、95% 分位绝对误差和最大绝对误差。`hardware_validated` 始终为 `false`，表示离线拟合工具不能认证实机效果。

结合独立工况的最坏低估误差设置功率裕量，而不能只看均方误差。若出现全零损耗、较大系统性偏差或跨温度/负载失效，应先补采数据。导出的 C 头文件包含 `lp_fitted_motor_model`；每台电机可以使用各自模型，不应把单电机空载损耗作为整车固定损耗重复或遗漏。

当前不自动进行在线 RLS 更新。没有一致的功率测量边界、足够激励和有效样本门控时，在线更新可能把电容充放电、延迟和噪声误学为电机参数。
