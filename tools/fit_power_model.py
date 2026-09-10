#!/usr/bin/env python3
"""Fit nonnegative motor losses from explicitly identified, aligned CSV data.

Copyright (c) 2026 LYHrmer. Distributed under the MIT license in license.txt.
Only Python's standard library is required. Mechanical coefficient is supplied,
not fitted, so current scaling and the shaft convention remain explicit.
"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sys


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("expected a finite nonnegative number")
    return number


def positive(value):
    number = nonnegative(value)
    if number == 0:
        raise argparse.ArgumentTypeError("expected a positive number")
    return number


def read_samples(paths, data_format, bus_voltage, max_skew_ms):
    samples = []
    records = []
    for path in paths:
        accepted = invalid = skewed = 0
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream, delimiter=";" if data_format == "legacy" else ",")
            required = (
                {"give_current", "motor_chassis[0].speed_rpm", "current",
                 "timestamp_current", "timestamp_give_current",
                 "timestamp_motor_chassis[0].speed_rpm"}
                if data_format == "legacy"
                else {"rpm", "current_command", "power_w"}
            )
            missing = required.difference(reader.fieldnames or [])
            if missing:
                raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
            for row in reader:
                try:
                    if data_format == "legacy":
                        rpm = float(row["motor_chassis[0].speed_rpm"])
                        command = float(row["give_current"])
                        power = float(row["current"]) * bus_voltage
                        timestamps = [float(row[key]) for key in (
                            "timestamp_current", "timestamp_give_current",
                            "timestamp_motor_chassis[0].speed_rpm")]
                        if not all(math.isfinite(value) for value in timestamps):
                            raise ValueError("nonfinite timestamp")
                        if max(timestamps) - min(timestamps) > max_skew_ms:
                            skewed += 1
                            continue
                    else:
                        rpm = float(row["rpm"])
                        command = float(row["current_command"])
                        power = float(row["power_w"])
                    if not all(math.isfinite(value) for value in (rpm, command, power)):
                        raise ValueError("nonfinite sample")
                    # Reject values that cannot be processed by this float C API.
                    if abs(rpm) > 1e7 or abs(command) > 1e7 or abs(power) > 1e9:
                        raise ValueError("sample outside supported numerical range")
                except (ValueError, TypeError, KeyError):
                    invalid += 1
                    continue
                samples.append((rpm, command, power))
                accepted += 1
        records.append({"file": str(path), "accepted_rows": accepted,
                        "invalid_rows": invalid, "misaligned_rows": skewed})
        if accepted == 0:
            raise ValueError(f"{path}: no valid aligned samples")
    return samples, records


def solve(matrix, rhs):
    """Small scaled normal equations, with rank detection and pivoting."""
    count = len(rhs)
    augmented = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for column in range(count):
        pivot = max(range(column, count), key=lambda index: abs(augmented[index][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            raise ValueError("insufficient independent excitation; collect varied speed/load data")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(count):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [a - factor * b for a, b in zip(augmented[row], augmented[column])]
    return [row[-1] for row in augmented]


def features(rpm, command, speed_model):
    speed = {"linear": [abs(rpm)], "quadratic": [rpm * rpm],
             "combined": [abs(rpm), rpm * rpm]}[speed_model]
    return speed + [command * command, 1.0]


def fit(samples, k_mech, speed_model, current_limit):
    names = {"linear": ["k_speed_abs"], "quadratic": ["k_speed_sq"],
             "combined": ["k_speed_abs", "k_speed_sq"]}[speed_model]
    names = names + ["k_current_sq", "p_idle"]
    count = len(names)
    if len(samples) < count * 3:
        raise ValueError(f"at least {count * 3} valid samples are required")
    raw = [features(rpm, command, speed_model) for rpm, command, _ in samples]
    scales = [max(abs(row[index]) for row in raw) for index in range(count)]
    if any(value == 0 for value in scales):
        raise ValueError("unexcited feature; collect varied speed/load data")
    rows = [[value / scale for value, scale in zip(row, scales)] for row in raw]
    targets = [power - k_mech * rpm * command for rpm, command, power in samples]
    size = len(rows)
    gram = [[math.fsum(row[i] * row[j] for row in rows) / size
             for j in range(count)] for i in range(count)]
    rhs = [math.fsum(row[i] * target for row, target in zip(rows, targets)) / size
           for i in range(count)]
    solve(gram, rhs)  # Fail explicitly if the requested model is unidentifiable.
    best = [0.0] * count
    best_cost = 0.0
    # There are at most four losses: enumerate active sets for exact small NNLS.
    for active_count in range(1, count + 1):
        for active in itertools.combinations(range(count), active_count):
            coefficients = solve([[gram[i][j] for j in active] for i in active],
                                 [rhs[i] for i in active])
            if any(value < -1e-9 for value in coefficients):
                continue
            candidate = [0.0] * count
            for index, value in zip(active, coefficients):
                candidate[index] = max(value, 0.0)
            cost = math.fsum(candidate[i] * gram[i][j] * candidate[j]
                             for i in range(count) for j in range(count))
            cost -= 2 * math.fsum(a * b for a, b in zip(candidate, rhs))
            if cost < best_cost:
                best, best_cost = candidate, cost
    model = {"k_mech": k_mech, "k_speed_abs": 0.0, "k_speed_sq": 0.0,
             "k_current_sq": 0.0, "p_idle": 0.0, "current_limit": current_limit}
    model.update({name: value / scale for name, value, scale in zip(names, best, scales)})
    if not all(math.isfinite(value) and 0 <= value <= 3.402823466e38 for value in model.values()):
        raise ValueError("fitted coefficients exceed the supported C float range")
    return model


def predict(model, rpm, command):
    return (model["k_mech"] * command * rpm + model["k_speed_abs"] * abs(rpm)
            + model["k_speed_sq"] * rpm * rpm
            + model["k_current_sq"] * command * command + model["p_idle"])


def metrics(samples, model):
    errors = [predict(model, rpm, command) - power for rpm, command, power in samples]
    absolute = sorted(abs(value) for value in errors)
    return {"samples": len(samples),
            "rmse_w": math.sqrt(math.fsum(value * value for value in errors) / len(errors)),
            "mae_w": math.fsum(absolute) / len(errors),
            "bias_w": math.fsum(errors) / len(errors),
            "p95_absolute_error_w": absolute[max(0, math.ceil(0.95 * len(absolute)) - 1)],
            "max_absolute_error_w": absolute[-1]}


def fit_warnings(model):
    losses = ("k_speed_abs", "k_speed_sq", "k_current_sq", "p_idle")
    if all(model[key] == 0 for key in losses):
        return ["All fitted losses are zero: check current scale/sign, assumed voltage, "
                "measurement boundary and alignment. Do not deploy this fit."]
    return []


def write_header(path, model):
    def c_float(value):
        token = format(value, ".9g")
        if "." not in token and "e" not in token:
            token += ".0"
        return token + "f"
    members = "\n".join(f"    .{name} = {c_float(value)}," for name, value in model.items())
    warning = "/* REJECT FOR DEPLOYMENT: all fitted losses are zero. */\n" if fit_warnings(model) else ""
    path.write_text(warning +
        "/* Generated fit; validate on your hardware before deployment. */\n"
        "#ifndef LP_FITTED_MOTOR_MODEL_H\n#define LP_FITTED_MOTOR_MODEL_H\n"
        '#include "ly_power/power_control.h"\n'
        "static const lp_motor_model_t lp_fitted_motor_model = {\n"
        + members + "\n};\n#endif\n", encoding="ascii")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+", help="training CSV files")
    parser.add_argument("--validation", type=Path, nargs="+", default=[], help="separate holdout CSV files")
    parser.add_argument("--format", choices=("canonical", "legacy"), default="canonical")
    parser.add_argument("--bus-voltage", type=positive, help="required for legacy current-to-power conversion")
    parser.add_argument("--k-mech", type=nonnegative, required=True, help="W / (raw command * rpm)")
    parser.add_argument("--current-limit", type=positive, required=True, help="raw command limit")
    parser.add_argument("--speed-model", choices=("linear", "quadratic", "combined"), default="quadratic")
    parser.add_argument("--max-skew-ms", type=nonnegative, default=5.0)
    parser.add_argument("--output", type=Path, required=True, help="fit and provenance JSON")
    parser.add_argument("--header", type=Path, help="optional C99 model header")
    args = parser.parse_args(argv)
    if args.format == "legacy" and args.bus_voltage is None:
        parser.error("--format legacy requires an explicit --bus-voltage assumption")
    train_paths = {path.resolve() for path in args.inputs}
    validation_paths = {path.resolve() for path in args.validation}
    if train_paths & validation_paths:
        parser.error("training and validation must use different files")
    if len(train_paths) != len(args.inputs) or len(validation_paths) != len(args.validation):
        parser.error("duplicate input files would bias the fit or evaluation")
    outputs = [path.resolve() for path in (args.output, args.header) if path is not None]
    if len(outputs) != len(set(outputs)) or set(outputs) & (train_paths | validation_paths):
        parser.error("outputs must be distinct from each other and from input datasets")
    try:
        samples, training_records = read_samples(args.inputs, args.format, args.bus_voltage, args.max_skew_ms)
        holdout, validation_records = read_samples(args.validation, args.format, args.bus_voltage, args.max_skew_ms)
        model = fit(samples, args.k_mech, args.speed_model, args.current_limit)
        report = {
            "schema_version": 1, "motor_model": model,
            "units": {"speed": "rpm", "current": "raw controller command", "power": "W"},
            "speed_model": args.speed_model, "training": metrics(samples, model),
            "validation": metrics(holdout, model) if holdout else None,
            "provenance": {"training_files": training_records, "validation_files": validation_records,
                           "format": args.format, "assumed_bus_voltage_v": args.bus_voltage,
                           "max_timestamp_skew_ms": args.max_skew_ms if args.format == "legacy" else None},
            "hardware_validated": False,
            "warnings": fit_warnings(model),
            "notes": ["One motor per row; power measurement must cover the same electrical boundary.",
                      "Losses are nonnegative; mechanical coefficient and current limit were supplied.",
                      "A holdout score is not evidence of closed-loop or competition performance."],
        }
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        if args.header:
            write_header(args.header, model)
        print(json.dumps({"model": model, "training": report["training"], "validation": report["validation"],
                          "warnings": report["warnings"]}, indent=2))
    except (OSError, ValueError, OverflowError) as error:
        parser.exit(2, f"fit failed: {error}\n")


if __name__ == "__main__":
    main()
