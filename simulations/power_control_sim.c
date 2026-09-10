/* Copyright (c) 2026 LYHrmer. SPDX-License-Identifier: MIT
 * Synthetic dynamic regression, not hardware calibration or competition proof.
 */
#include "ly_power/power_control.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_COUNT 4u
#define STEP_S 0.002
#define DURATION_S 28.0
#define PI_VALUE 3.14159265358979323846
#define AMPS_PER_RAW (20.0 / 16384.0)
#define CAP_CAPACITY_J 90.0
#define BUFFER_CAPACITY_J 40.0
#define TELEMETRY_PERIOD 10u
#define TELEMETRY_TIMEOUT_S 0.10

typedef struct {
    double torque_constant, inertia, viscous, resistance;
    double speed_abs, speed_sq, idle, current_limit;
} plant_motor_t;

typedef struct {
    const char *name;
    double target_rpm, referee_limit_w, load_nm;
    lp_mode_t mode;
    int freeze_referee, freeze_buffer, freeze_cap;
} scenario_t;

typedef struct {
    double omega[MOTOR_COUNT], integral[MOTOR_COUNT];
    double cap_energy, buffer_energy;
    double last_referee, last_buffer, last_cap;
    double sampled_referee, sampled_buffer, sampled_cap;
    double squared_tracking_error, load_energy, source_energy;
    double load_excess_energy, source_excess_energy;
    double peak_load_excess, peak_source_excess, peak_model_excess;
    double peak_load_power, squared_model_error, peak_absolute_model_error;
    unsigned long limited_steps, infeasible_steps;
    uint32_t seen_budget_flags, seen_allocation_flags;
} run_t;

static const plant_motor_t physical[MOTOR_COUNT] = {
    {0.100, 0.022, 0.0055, 0.100, 0.0010, 0.0000030, 0.55, 13000.0},
    {0.115, 0.024, 0.0062, 0.105, 0.0012, 0.0000035, 0.60, 14500.0},
    {0.105, 0.020, 0.0048, 0.110, 0.0011, 0.0000040, 0.50, 13800.0},
    {0.110, 0.025, 0.0059, 0.095, 0.0013, 0.0000032, 0.65, 15000.0}
};
static const double target_factor[MOTOR_COUNT] = {0.95, 1.05, 1.02, 0.98};
static const lp_budget_config_t budget_config = {
    35.0f, 10.0f, 15.0f, 20.0f, 1.0f, 12.0f, 65.0f, 0.6f, 180.0f
};

static double clip(double value, double lower, double upper)
{
    return fmax(lower, fmin(upper, value));
}

static scenario_t scenario_at(double time)
{
    scenario_t s = {"startup", 180.0, 90.0, 0.12,
                    LP_MODE_NORMAL, 0, 0, 0};
    if (time < 0.5) s.target_rpm = 0.0;
    if (time >= 2.0) { s.name = "cruise"; s.target_rpm = 260.0; }
    if (time >= 5.0) { s.name = "load_step"; s.load_nm = 0.65; }
    if (time >= 8.0) { s.name = "judge_step"; s.referee_limit_w = 45.0; }
    if (time >= 11.0) {
        s.name = "reverse"; s.target_rpm = -220.0; s.referee_limit_w = 75.0;
    }
    if (time >= 14.0) {
        s.name = "charge"; s.target_rpm = 120.0; s.mode = LP_MODE_CHARGE;
    }
    if (time >= 16.0) {
        s.name = "boost"; s.target_rpm = 380.0; s.mode = LP_MODE_BOOST;
    }
    if (time >= 19.0) s.name = "cap_reserve";
    if (time >= 21.0) {
        s.name = "cap_stale"; s.target_rpm = 220.0;
        s.load_nm = 0.28; s.freeze_cap = 1;
    }
    if (time >= 23.0) {
        s.name = "referee_stale"; s.target_rpm = 180.0;
        s.freeze_cap = 0; s.freeze_referee = 1;
    }
    if (time >= 25.0) {
        s.name = "buffer_stale"; s.target_rpm = 240.0;
        s.freeze_referee = 0; s.freeze_buffer = 1;
    }
    if (time >= 27.0) {
        s.name = "recovery"; s.target_rpm = 160.0;
        s.mode = LP_MODE_NORMAL; s.freeze_buffer = 0;
    }
    return s;
}

static void initialize_run(run_t *run)
{
    memset(run, 0, sizeof(*run));
    run->cap_energy = 70.0;
    run->buffer_energy = 35.0;
    run->sampled_cap = run->cap_energy;
    run->sampled_buffer = run->buffer_energy;
    run->sampled_referee = 90.0;
}

static void make_models(lp_motor_model_t models[MOTOR_COUNT])
{
    size_t i;
    for (i = 0; i < MOTOR_COUNT; ++i) {
        const plant_motor_t *p = &physical[i];
        /* Deliberate 8-11% underestimation. No measured calibration is claimed. */
        double bias = 0.89 + 0.01 * (double)i;
        models[i].k_mech = (float)(bias * p->torque_constant *
                                  AMPS_PER_RAW * 2.0 * PI_VALUE / 60.0);
        models[i].k_speed_abs = (float)(bias * p->speed_abs);
        models[i].k_speed_sq = (float)(bias * p->speed_sq);
        models[i].k_current_sq = (float)(bias * p->resistance *
                                        AMPS_PER_RAW * AMPS_PER_RAW);
        models[i].p_idle = (float)(bias * p->idle);
        models[i].current_limit = (float)p->current_limit;
    }
}

static lp_budget_t update_budget(run_t *run, const scenario_t *scenario,
                                 double time, unsigned long tick)
{
    lp_budget_input_t input;
    lp_budget_t budget;
    if (tick % TELEMETRY_PERIOD == 0u) {
        if (!scenario->freeze_referee) {
            run->sampled_referee = scenario->referee_limit_w;
            run->last_referee = time;
        }
        if (!scenario->freeze_buffer) {
            run->sampled_buffer = run->buffer_energy;
            run->last_buffer = time;
        }
        if (!scenario->freeze_cap) {
            run->sampled_cap = run->cap_energy;
            run->last_cap = time;
        }
    }
    input.judge_limit_w = (float)run->sampled_referee;
    input.buffer_energy_j = (float)run->sampled_buffer;
    input.cap_energy_j = (float)run->sampled_cap;
    input.judge_valid = time - run->last_referee <= TELEMETRY_TIMEOUT_S;
    input.buffer_valid = time - run->last_buffer <= TELEMETRY_TIMEOUT_S;
    input.cap_valid = time - run->last_cap <= TELEMETRY_TIMEOUT_S;
    input.requested_mode = scenario->mode;
    (void)lp_compute_budget(&budget_config, &input, &budget);
    return budget;
}

/* Idealized downstream capacitor with an autonomous battery-side threshold.
 * If discharge power/energy is insufficient, the source supplies the remainder;
 * source power can then exceed the chosen referee threshold and drain buffer.
 * No recovered braking power is credited. Charging/discharging efficiency is 1.
 */
static double update_supply(run_t *run, double load_power, double source_limit)
{
    double charge = 0.0, discharge = 0.0, source_power;
    if (load_power > source_limit) {
        discharge = fmin(load_power - source_limit, 65.0);
        discharge = fmin(discharge, run->cap_energy / STEP_S);
    } else {
        charge = fmin(source_limit - load_power, 15.0);
        charge = fmin(charge, (CAP_CAPACITY_J - run->cap_energy) / STEP_S);
    }
    source_power = load_power + charge - discharge;
    run->cap_energy = clip(run->cap_energy + (charge - discharge) * STEP_S,
                           0.0, CAP_CAPACITY_J);
    run->buffer_energy = clip(run->buffer_energy +
                              (source_limit - source_power) * STEP_S,
                              0.0, BUFFER_CAPACITY_J);
    return source_power;
}

static int fail(double time, const char *run_name, const char *message)
{
    fprintf(stderr, "regression failed at %.6fs (%s): %s\n",
            time, run_name, message);
    return 0;
}

static int step_run(run_t *run, int controlled,
                    const lp_motor_model_t models[MOTOR_COUNT],
                    const scenario_t *scenario, double time,
                    unsigned long tick, FILE *csv)
{
    const char *name = controlled ? "controlled" : "unlimited";
    const uint32_t errors = LP_STATUS_INVALID_ARGUMENT | LP_STATUS_INVALID_MODEL |
                            LP_STATUS_NUMERIC_ERROR | LP_STATUS_NEGATIVE_BUDGET;
    lp_budget_t budget = update_budget(run, scenario, time, tick);
    lp_allocation_t allocation;
    float rpm[MOTOR_COUNT], requests[MOTOR_COUNT];
    double errors_rpm[MOTOR_COUNT], power = 0.0, mean_rpm = 0.0;
    double tracking_sq = 0.0, max_current = 0.0, source_power;
    double cap_before = run->cap_energy, buffer_before = run->buffer_energy;
    size_t i;
    for (i = 0; i < MOTOR_COUNT; ++i) {
        rpm[i] = (float)(run->omega[i] * 60.0 / (2.0 * PI_VALUE));
        errors_rpm[i] = scenario->target_rpm * target_factor[i] - rpm[i];
        requests[i] = (float)(100.0 * errors_rpm[i] + run->integral[i]);
        mean_rpm += rpm[i] / (double)MOTOR_COUNT;
        tracking_sq += errors_rpm[i] * errors_rpm[i] / (double)MOTOR_COUNT;
    }
    /* The baseline keeps identical current limits but has no power restriction. */
    (void)lp_allocate(models, rpm, requests, MOTOR_COUNT,
                      controlled ? budget.budget_w : 1000000.0f, &allocation);
    if ((budget.flags | allocation.flags) & errors)
        return fail(time, name, "core reported an invalid or numeric result");
    if (!isfinite(budget.budget_w) || !isfinite(allocation.predicted_power_w) ||
        !isfinite(allocation.scale) || allocation.scale < 0.0f ||
        allocation.scale > 1.0f || allocation.count != MOTOR_COUNT)
        return fail(time, name, "non-finite power, invalid scale, or motor count");
    if (controlled && !(allocation.flags & LP_STATUS_INFEASIBLE) &&
        allocation.predicted_power_w > budget.budget_w + 0.002f)
        return fail(time, name, "feasible allocation exceeds model budget");

    for (i = 0; i < MOTOR_COUNT; ++i) {
        const plant_motor_t *p = &physical[i];
        double command = allocation.allocated_current[i];
        double amps = command * AMPS_PER_RAW;
        double torque = p->torque_constant * amps;
        double load_torque = scenario->load_nm * (1.0 + 0.04 * (double)i) *
                             tanh(run->omega[i] / 0.5);
        double motor_power = torque * run->omega[i] + p->resistance * amps * amps +
                             p->speed_abs * fabs(rpm[i]) +
                             p->speed_sq * rpm[i] * rpm[i] + p->idle;
        double command_limit = fmin(fabs(requests[i]), p->current_limit);
        if (!isfinite(command) || fabs(command) > command_limit + 0.01 ||
            command * requests[i] < -0.001)
            return fail(time, name, "current non-finite, amplified, or sign reversed");
        power += fmax(0.0, motor_power);
        max_current = fmax(max_current, fabs(command));
        /* Independent mechanical state, advanced from actual allocated current. */
        run->omega[i] += STEP_S * (torque - p->viscous * run->omega[i] -
                                   load_torque) / p->inertia;
        /* Back calculation releases the integral under either current or power clamp. */
        run->integral[i] = clip(run->integral[i] + STEP_S *
            (120.0 * errors_rpm[i] + 12.0 * (command - requests[i])),
            -p->current_limit, p->current_limit);
        if (!isfinite(run->omega[i]) || !isfinite(run->integral[i]))
            return fail(time, name, "non-finite dynamic state");
    }
    source_power = update_supply(run, power, scenario->referee_limit_w);
    if (!isfinite(power) || !isfinite(source_power) ||
        !isfinite(run->cap_energy) || !isfinite(run->buffer_energy) ||
        run->cap_energy < 0.0 || run->cap_energy > CAP_CAPACITY_J ||
        run->buffer_energy < 0.0 || run->buffer_energy > BUFFER_CAPACITY_J)
        return fail(time, name, "power or energy state outside finite bounds");

    run->squared_tracking_error += tracking_sq * STEP_S;
    run->peak_load_power = fmax(run->peak_load_power, power);
    run->squared_model_error += (power - allocation.predicted_power_w) *
                               (power - allocation.predicted_power_w) * STEP_S;
    run->peak_absolute_model_error = fmax(run->peak_absolute_model_error,
                                         fabs(power - allocation.predicted_power_w));
    run->load_energy += power * STEP_S;
    run->source_energy += source_power * STEP_S;
    run->load_excess_energy += fmax(0.0, power - budget.budget_w) * STEP_S;
    run->source_excess_energy +=
        fmax(0.0, source_power - scenario->referee_limit_w) * STEP_S;
    run->peak_load_excess = fmax(run->peak_load_excess, power - budget.budget_w);
    run->peak_source_excess = fmax(run->peak_source_excess,
                                  source_power - scenario->referee_limit_w);
    if (!(allocation.flags & LP_STATUS_INFEASIBLE))
        run->peak_model_excess = fmax(run->peak_model_excess,
                              allocation.predicted_power_w - budget.budget_w);
    if (allocation.flags & LP_STATUS_POWER_LIMITED) run->limited_steps++;
    if (allocation.flags & LP_STATUS_INFEASIBLE) run->infeasible_steps++;
    run->seen_budget_flags |= budget.flags;
    run->seen_allocation_flags |= allocation.flags;

    if (csv) {
        if (fprintf(csv,
            "%.6f,%s,%s,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%u,%u,%.9g\n",
            time, scenario->name, name, (int)budget.effective_mode,
            (double)budget.budget_w, (double)allocation.predicted_power_w,
            power, source_power, scenario->referee_limit_w,
            cap_before, buffer_before, mean_rpm, scenario->target_rpm,
            sqrt(tracking_sq), (double)allocation.scale,
            (unsigned)budget.flags, (unsigned)allocation.flags, max_current) < 0)
            return fail(time, name, "CSV write failed");
    }
    return 1;
}

static void print_statistics(const char *name, const run_t *run)
{
    printf("%s: tracking_rmse=%.3f rpm, peak_plant_power=%.3f W, "
           "power_model_rmse=%.3f W, peak_model_error=%.3f W, "
           "peak_load_over_budget=%.3f W, "
           "peak_source_over_referee=%.3f W, source_excess=%.3f J, "
           "limited_steps=%lu, infeasible_steps=%lu, budget_flags=0x%03x\n",
           name, sqrt(run->squared_tracking_error / DURATION_S),
           run->peak_load_power, sqrt(run->squared_model_error / DURATION_S),
           run->peak_absolute_model_error,
           run->peak_load_excess, run->peak_source_excess,
           run->source_excess_energy, run->limited_steps, run->infeasible_steps,
           (unsigned)run->seen_budget_flags);
}

int main(int argc, char **argv)
{
    const uint32_t required_budget = LP_STATUS_JUDGE_STALE | LP_STATUS_BUFFER_STALE |
                                    LP_STATUS_CAP_STALE | LP_STATUS_MODE_DEGRADED;
    const uint32_t required_allocation = LP_STATUS_POWER_LIMITED |
                                        LP_STATUS_CURRENT_CLAMPED;
    const unsigned long ticks = (unsigned long)(DURATION_S / STEP_S + 0.5);
    int check_only = argc == 2 && strcmp(argv[1], "--check-only") == 0;
    const char *path = argc == 2 ? argv[1] : "power_control_sim.csv";
    lp_motor_model_t models[MOTOR_COUNT];
    run_t controlled, baseline;
    FILE *csv = NULL;
    unsigned long tick;
    if (argc > 2) {
        fprintf(stderr, "usage: %s [output.csv | --check-only]\n", argv[0]);
        return EXIT_FAILURE;
    }
    if (!check_only) {
        csv = fopen(path, "w");
        if (!csv) { perror(path); return EXIT_FAILURE; }
        fprintf(csv, "time_s,scenario,run,mode,budget_w,model_power_w,plant_power_w,"
                     "source_power_w,referee_limit_w,cap_energy_j,buffer_energy_j,"
                     "mean_rpm,target_rpm,tracking_error_rpm,scale,budget_flags,"
                     "allocation_flags,max_abs_current_raw\n");
    }
    make_models(models);
    initialize_run(&controlled);
    initialize_run(&baseline);
    for (tick = 0; tick < ticks; ++tick) {
        double time = (double)tick * STEP_S;
        scenario_t scenario = scenario_at(time);
        if (!step_run(&controlled, 1, models, &scenario, time, tick, csv) ||
            !step_run(&baseline, 0, models, &scenario, time, tick, csv)) {
            if (csv) fclose(csv);
            return EXIT_FAILURE;
        }
    }
    if (csv && fclose(csv) != 0) { perror(path); return EXIT_FAILURE; }
    if ((controlled.seen_budget_flags & required_budget) != required_budget ||
        (controlled.seen_allocation_flags & required_allocation) != required_allocation) {
        fprintf(stderr, "regression failed: expected stale/degraded/limit flags missing\n");
        return EXIT_FAILURE;
    }
    print_statistics("controlled", &controlled);
    print_statistics("unlimited", &baseline);
    printf("PASS: finite dynamics, command bounds/sign, feasible model budget, "
           "energy bounds, and stale/degraded coverage (%lu ticks per run).\n", ticks);
    printf("Physical excess is reported, not treated as a guaranteed hardware pass.\n");
    if (csv) printf("CSV: %s\n", path);
    return EXIT_SUCCESS;
}
