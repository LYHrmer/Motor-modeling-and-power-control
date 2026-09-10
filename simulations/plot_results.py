#!/usr/bin/env python3
"""Plot synthetic closed-loop results without pandas. Copyright 2026 LYHrmer, MIT."""
import argparse
import csv
import math
from pathlib import Path
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_runs(path):
    runs = {"controlled": [], "unlimited": []}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            run = row.pop("run")
            scenario = row.pop("scenario")
            if run not in runs:
                raise ValueError("Unknown run: " + run)
            values = {key: float(value) for key, value in row.items()}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("Non-finite CSV sample")
            values["scenario"] = scenario
            runs[run].append(values)
    for name, rows in runs.items():
        if len(rows) < 2:
            raise ValueError("Insufficient samples for " + name)
        if any(b["time_s"] <= a["time_s"] for a, b in zip(rows, rows[1:])):
            raise ValueError("Time must increase for " + name)
    return runs


def summarize(rows):
    dt = statistics.median(b["time_s"] - a["time_s"] for a, b in zip(rows, rows[1:]))
    durations = [b["time_s"] - a["time_s"] for a, b in zip(rows, rows[1:])] + [dt]
    duration = sum(durations)
    integral = lambda key: sum(r[key] * d for r, d in zip(rows, durations))
    excess = lambda a, b: sum(max(0.0, r[a] - r[b]) * d for r, d in zip(rows, durations))
    flags = 0
    for row in rows:
        flags |= int(row["budget_flags"])
    return {
        "duration": duration,
        "rmse": math.sqrt(sum(r["tracking_error_rpm"] ** 2 * d
                              for r, d in zip(rows, durations)) / duration),
        "load_j": integral("plant_power_w"),
        "source_j": integral("source_power_w"),
        "peak_plant_power": max(r["plant_power_w"] for r in rows),
        "model_rmse": math.sqrt(sum((r["plant_power_w"] - r["model_power_w"]) ** 2 * d
                                    for r, d in zip(rows, durations)) / duration),
        "peak_model_error": max(abs(r["plant_power_w"] - r["model_power_w"]) for r in rows),
        "peak_load_excess": max(max(0.0, r["plant_power_w"] - r["budget_w"]) for r in rows),
        "peak_source_excess": max(max(0.0, r["source_power_w"] - r["referee_limit_w"]) for r in rows),
        "load_excess_j": excess("plant_power_w", "budget_w"),
        "source_excess_j": excess("source_power_w", "referee_limit_w"),
        "min_cap": min(r["cap_energy_j"] for r in rows),
        "min_buffer": min(r["buffer_energy_j"] for r in rows),
        "limited_percent": 100.0 * sum(d for r, d in zip(rows, durations)
                                       if int(r["allocation_flags"]) & (1 << 4)) / duration,
        "infeasible_samples": sum(bool(int(r["allocation_flags"]) & (1 << 5)) for r in rows),
        "budget_flags": flags,
    }


def make_figure(runs, output_dir):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "#f6f7f4", "axes.facecolor": "#ffffff",
                         "savefig.facecolor": "#f6f7f4", "svg.fonttype": "none"})
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True)
    ax = axes.flat
    colors = {"controlled": "#087f8c", "unlimited": "#c45a2a"}
    labels = {"controlled": "Power controlled", "unlimited": "Current limits only"}
    for name, rows in runs.items():
        time = [r["time_s"] for r in rows]
        color = colors[name]
        for index, key in [(0, "plant_power_w"), (1, "source_power_w"),
                           (2, "mean_rpm"), (3, "cap_energy_j"),
                           (4, "buffer_energy_j"), (5, "scale")]:
            ax[index].plot(time, [r[key] for r in rows], color=color,
                           linewidth=1.2, label=labels[name], alpha=0.9)
    controlled = runs["controlled"]
    time = [r["time_s"] for r in controlled]
    ax[0].plot(time, [r["budget_w"] for r in controlled], color="#273444",
               linewidth=1.0, linestyle="--", label="Controlled budget")
    ax[0].plot(time, [r["model_power_w"] for r in controlled], color="#5da5a8",
               linewidth=0.7, linestyle=":", label="Controller prediction")
    ax[1].plot(time, [r["referee_limit_w"] for r in controlled], color="#273444",
               linewidth=1.0, linestyle="--", label="Source threshold")
    ax[2].plot(time, [r["target_rpm"] for r in controlled], color="#273444",
               linewidth=1.0, linestyle="--", label="Mean target")
    ax[3].axhline(12.0, color="#273444", linewidth=0.8, linestyle="--", label="Boost reserve")
    ax[4].axhline(20.0, color="#273444", linewidth=0.8, linestyle="--", label="Buffer target")
    titles = ["A. Motor-side power: model error remains", "B. Battery-side power",
              "C. Independent motor dynamics", "D. Usable capacitor energy",
              "E. Synthetic referee buffer", "F. Common allocation scale"]
    units = ["Power [W]", "Power [W]", "Mean speed [rpm]", "Energy [J]", "Energy [J]", "Scale"]
    boundaries = [2, 5, 8, 11, 14, 16, 19, 21, 23, 25, 27]
    for index, axis in enumerate(axes.flat):
        axis.set_title(titles[index], loc="left", fontweight="bold", pad=10)
        axis.set_ylabel(units[index])
        axis.grid(alpha=0.15)
        axis.set_xlim(0, 28)
        for boundary in boundaries:
            axis.axvline(boundary, linewidth=0.5, color="#9da8ad", alpha=0.35)
        axis.legend(loc="best", frameon=False, fontsize=8)
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 1].set_xlabel("Time [s]")
    fig.suptitle("POWER CONTROL / DYNAMIC REGRESSION", x=0.075, ha="left",
                 fontsize=20, fontweight="bold", color="#17383e")
    fig.text(0.075, 0.943,
             "Synthetic heterogeneous motors | 8-11% model underestimation | "
             "Independent controlled and baseline states | Not hardware evidence",
             fontsize=10, color="#58676b")
    fig.tight_layout(rect=(0.01, 0.015, 0.99, 0.915), h_pad=2.4, w_pad=2.0)
    for suffix in ("png", "svg"):
        fig.savefig(output_dir / ("power_control_sim." + suffix), dpi=180)
    plt.close(fig)


def write_summary(runs, output_dir, source):
    values = {name: summarize(rows) for name, rows in runs.items()}
    rows = [
        ("速度跟踪 RMSE / rpm", "rmse"),
        ("实际电机侧功率峰值 / W", "peak_plant_power"),
        ("功率模型 RMSE / W", "model_rmse"),
        ("功率模型最大绝对误差 / W", "peak_model_error"),
        ("电机侧消耗能量 / J", "load_j"),
        ("电池侧消耗能量 / J", "source_j"),
        ("实际电机功率高于各自预算的最大值 / W", "peak_load_excess"),
        ("实际电机功率高于各自预算的积分 / J", "load_excess_j"),
        ("电池侧功率高于供电阈值的最大值 / W", "peak_source_excess"),
        ("电池侧超阈值积分 / J", "source_excess_j"),
        ("最小电容可用能量 / J", "min_cap"),
        ("最小缓冲能量 / J", "min_buffer"),
        ("功率限幅时间占比 / %", "limited_percent"),
        ("模型不可行采样数", "infeasible_samples"),
    ]
    text = ["# 动态仿真结果", "",
            "本结果来自合成参数和独立动态电机模型，不是实车采集、比赛结果或硬件认证。", "",
            "输入 CSV：`" + str(source) + "`。两条轨迹均运行 {:.3f} 秒。".format(values["controlled"]["duration"]), "",
            "| 指标 | 限功率控制 | 仅限流基线 |", "| --- | ---: | ---: |"]
    for label, key in rows:
        text.append("| {} | {:.3f} | {:.3f} |".format(label, values["controlled"][key], values["unlimited"][key]))
    text += ["", "## 结果解读", "",
             "控制器模型故意比物理 plant 低估约 8% 至 11%。因此模型内满足预算，不代表实际电机侧功率严格低于预算；表格保留真实超调，不将其掩盖或作为自动通过指标。", "",
             "基线仅保留各电机电流限制，使用独立电机、电容和缓冲状态。其日志中的预算是同一预算策略对基线自身状态计算出的参考值，基线不执行这个功率约束。", "",
             "电机侧预算与电池侧阈值不同；BOOST 可以使用电容供能。判断供电边界应同时看电池侧功率和能量变化。缓冲能量按假设饱和截断，零缓冲本身不会终止仿真。", "",
             "限功率后的速度误差可能增加，这是功率约束的控制代价。两条轨迹负载相同，但实际转速不同，因此消耗的机械能也不同，不能直接把总能量差叫作效率提升。", "",
             "控制轨迹覆盖预算 flags：`0x{:03x}`；基线覆盖：`0x{:03x}`。flags 的定义见 public header。".format(
                 values["controlled"]["budget_flags"], values["unlimited"]["budget_flags"]), "",
             "## 工况", "", "| 开始 / s | 场景 |", "| ---: | --- |"]
    last = None
    for row in runs["controlled"]:
        if row["scenario"] != last:
            text.append("| {:.3f} | `{}` |".format(row["time_s"], row["scenario"]))
            last = row["scenario"]
    text += ["", "![动态仿真曲线](power_control_sim.png)", "",
             "矢量图：`power_control_sim.svg`。完整模型、时间步长、供电边界与回归断言见 `simulations/README.md`。", ""]
    (output_dir / "summary.md").write_text("\n".join(text), encoding="utf-8")
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    runs = load_runs(args.csv_path)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    make_figure(runs, args.output_dir)
    stats = write_summary(runs, args.output_dir, args.csv_path)
    for name in ("controlled", "unlimited"):
        print("{}: tracking RMSE {:.3f} rpm; source excess {:.3f} J".format(
            name, stats[name]["rmse"], stats[name]["source_excess_j"]))
    print("Artifacts: " + str(args.output_dir / "power_control_sim.png"))
    print("Artifacts: " + str(args.output_dir / "power_control_sim.svg"))
    print("Artifacts: " + str(args.output_dir / "summary.md"))


if __name__ == "__main__":
    main()
