/* Copyright (c) 2026 LYHrmer. SPDX-License-Identifier: MIT
 * See the repository license.txt for the license text.
 */
#include "ly_power/power_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* The large bound covers extreme finite float coefficients. Most searches
 * finish after about 54 iterations when adjacent double endpoints meet.
 */
#define LP_SEARCH_STEPS 512u

typedef struct {
    size_t count;
    const lp_motor_model_t *models;
    const float *rpm;
    float current[LP_MAX_MOTORS];
    double a[LP_MAX_MOTORS];
    double b[LP_MAX_MOTORS];
    double c[LP_MAX_MOTORS];
} lp_problem_t;

static int lp_nonnegative(float x)
{
    return isfinite(x) && x >= 0.0f;
}

static float lp_power_float(double power, uint32_t *flags)
{
    float result;
    if (!isfinite(power) || power > (double)FLT_MAX) {
        *flags |= LP_STATUS_NUMERIC_ERROR;
        return FLT_MAX;
    }
    if (power <= 0.0) {
        return 0.0f;
    }
    result = (float)power;
    if ((double)result < power) {
        result = nextafterf(result, FLT_MAX);
    }
    return result;
}

static float lp_budget_float(double power)
{
    float result;
    if (power <= 0.0) {
        return 0.0f;
    }
    if (power >= (double)FLT_MAX) {
        return FLT_MAX;
    }
    result = (float)power;
    if ((double)result > power) {
        result = nextafterf(result, 0.0f);
    }
    return result;
}

static int lp_model_valid(const lp_motor_model_t *model)
{
    return lp_nonnegative(model->k_mech)
        && lp_nonnegative(model->k_speed_abs)
        && lp_nonnegative(model->k_speed_sq)
        && lp_nonnegative(model->k_current_sq)
        && lp_nonnegative(model->p_idle)
        && isfinite(model->current_limit) && model->current_limit > 0.0f;
}

/* max(convex quadratic, 0) is convex. At a kink its right derivative is
 * max(quadratic derivative, 0), which keeps the derivative monotonic.
 */
static double lp_ray_power(const lp_problem_t *problem, double scale,
                           double *right_derivative)
{
    double power = 0.0;
    double derivative = 0.0;
    size_t i;
    for (i = 0; i < problem->count; ++i) {
        double value = (problem->a[i] * scale + problem->b[i]) * scale
                     + problem->c[i];
        double slope = 2.0 * problem->a[i] * scale + problem->b[i];
        if (value > 0.0) {
            power += value;
            derivative += slope;
        } else if (value == 0.0 && slope > 0.0) {
            derivative += slope;
        }
    }
    if (right_derivative != NULL) {
        *right_derivative = derivative;
    }
    return power;
}

static double lp_minimum_scale(const lp_problem_t *problem)
{
    double low = 0.0;
    double high = 1.0;
    double derivative;
    unsigned int step;
    (void)lp_ray_power(problem, low, &derivative);
    if (derivative >= 0.0) {
        return low;
    }
    (void)lp_ray_power(problem, high, &derivative);
    if (derivative <= 0.0) {
        return high;
    }
    for (step = 0; step < LP_SEARCH_STEPS; ++step) {
        double middle = low + (high - low) * 0.5;
        if (middle == low || middle == high) {
            break;
        }
        (void)lp_ray_power(problem, middle, &derivative);
        if (derivative < 0.0) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return lp_ray_power(problem, low, NULL) < lp_ray_power(problem, high, NULL)
         ? low : high;
}

/* Evaluate the actual rounded output commands, not just the scalar polynomial.
 * With finite float inputs all triple products fit in double, including at
 * FLT_MAX. The final float power conversion checks its narrower output range.
 */
static double lp_command_power(const lp_problem_t *problem, float scale,
                               float *commands)
{
    double total = 0.0;
    size_t i;
    for (i = 0; i < problem->count; ++i) {
        float command = (float)((double)scale * (double)problem->current[i]);
        double current = (double)command;
        double power = (double)problem->models[i].k_current_sq * current * current
                     + (double)problem->models[i].k_mech * current
                       * (double)problem->rpm[i]
                     + problem->c[i];
        if (commands != NULL) {
            commands[i] = command;
        }
        if (power > 0.0) {
            total += power;
        }
    }
    return total;
}

/* Check adjacent float scales as well: a real-valued minimum may fall between
 * representable scales, and braking makes rounding toward zero insufficient.
 */
static float lp_representable_minimum(const lp_problem_t *problem,
                                     double minimum, double *power)
{
    float best = (float)minimum;
    float candidates[3];
    unsigned int i;
    candidates[0] = best;
    candidates[1] = nextafterf(best, 0.0f);
    candidates[2] = nextafterf(best, 1.0f);
    *power = lp_command_power(problem, best, NULL);
    for (i = 1; i < 3u; ++i) {
        double candidate_power = lp_command_power(problem, candidates[i], NULL);
        if (candidate_power < *power
            || (candidate_power == *power && candidates[i] > best)) {
            *power = candidate_power;
            best = candidates[i];
        }
    }
    return best;
}

uint32_t lp_allocate(const lp_motor_model_t *models, const float *rpm,
                     const float *requested_current, size_t count,
                     float budget_w, lp_allocation_t *out)
{
    lp_problem_t problem;
    uint32_t flags = LP_STATUS_OK;
    double minimum;
    double minimum_power;
    double minimum_command_power;
    double requested_power;
    double predicted_power;
    double budget;
    float minimum_command_scale;
    float scale;
    size_t i;
    unsigned int step;

    if (out == NULL) {
        return LP_STATUS_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (models == NULL || rpm == NULL || requested_current == NULL
        || count == 0u || count > LP_MAX_MOTORS || !isfinite(budget_w)) {
        out->flags = LP_STATUS_INVALID_ARGUMENT;
        return out->flags;
    }
    for (i = 0; i < count; ++i) {
        if (!lp_model_valid(&models[i])) {
            out->flags = LP_STATUS_INVALID_MODEL;
            return out->flags;
        }
        if (!isfinite(rpm[i]) || !isfinite(requested_current[i])) {
            out->flags = LP_STATUS_INVALID_ARGUMENT;
            return out->flags;
        }
    }
    budget = (double)budget_w;
    if (budget < 0.0) {
        budget = 0.0;
        flags |= LP_STATUS_NEGATIVE_BUDGET;
    }
    problem.count = count;
    problem.models = models;
    problem.rpm = rpm;
    for (i = 0; i < count; ++i) {
        double current;
        double speed = (double)rpm[i];
        problem.current[i] = requested_current[i];
        if (problem.current[i] > models[i].current_limit) {
            problem.current[i] = models[i].current_limit;
            flags |= LP_STATUS_CURRENT_CLAMPED;
        } else if (problem.current[i] < -models[i].current_limit) {
            problem.current[i] = -models[i].current_limit;
            flags |= LP_STATUS_CURRENT_CLAMPED;
        }
        current = (double)problem.current[i];
        problem.a[i] = (double)models[i].k_current_sq * current * current;
        problem.b[i] = (double)models[i].k_mech * current * speed;
        problem.c[i] = (double)models[i].k_speed_abs * fabs(speed)
                     + (double)models[i].k_speed_sq * speed * speed
                     + (double)models[i].p_idle;
        if (!isfinite(problem.a[i]) || !isfinite(problem.b[i])
            || !isfinite(problem.c[i])) {
            out->flags = flags | LP_STATUS_NUMERIC_ERROR;
            return out->flags;
        }
    }
    out->count = count;
    requested_power = lp_command_power(&problem, 1.0f, NULL);
    out->requested_power_w = lp_power_float(requested_power, &flags);
    if (requested_power <= budget) {
        scale = 1.0f;
    } else {
        minimum = lp_minimum_scale(&problem);
        minimum_power = lp_ray_power(&problem, minimum, NULL);
        minimum_command_scale = lp_representable_minimum(
            &problem, minimum, &minimum_command_power);
        if (minimum_power > budget && minimum_command_power > budget) {
            scale = minimum_command_scale;
            flags |= LP_STATUS_INFEASIBLE;
        } else {
            double low = minimum;
            double high = 1.0;
            if (minimum_power <= budget) {
                for (step = 0; step < LP_SEARCH_STEPS; ++step) {
                    double middle = low + (high - low) * 0.5;
                    if (middle == low || middle == high) {
                        break;
                    }
                    if (lp_ray_power(&problem, middle, NULL) <= budget) {
                        low = middle;
                    } else {
                        high = middle;
                    }
                }
                scale = (float)low;
            } else {
                scale = minimum_command_scale;
            }
            if (lp_command_power(&problem, scale, NULL) > budget) {
                if (minimum_command_power > budget) {
                    scale = minimum_command_scale;
                    flags |= LP_STATUS_INFEASIBLE | LP_STATUS_NUMERIC_ERROR;
                } else {
                    /* Keep an actually feasible float endpoint, correcting
                     * rounding at the power boundary toward the minimum.
                     */
                    float feasible = minimum_command_scale;
                    float infeasible = scale;
                    for (step = 0; step < LP_SEARCH_STEPS; ++step) {
                        float middle = (float)(((double)feasible
                                             + (double)infeasible) * 0.5);
                        if (middle == feasible || middle == infeasible) {
                            break;
                        }
                        if (lp_command_power(&problem, middle, NULL) <= budget) {
                            feasible = middle;
                        } else {
                            infeasible = middle;
                        }
                    }
                    scale = feasible;
                }
            }
        }
    }
    out->scale = scale;
    if (scale < 1.0f) {
        flags |= LP_STATUS_POWER_LIMITED;
    }
    predicted_power = lp_command_power(&problem, scale, out->allocated_current);
    if (predicted_power > budget) {
        flags |= LP_STATUS_INFEASIBLE;
    }
    out->predicted_power_w = lp_power_float(predicted_power, &flags);
    out->flags = flags;
    return flags;
}

static int lp_budget_config_valid(const lp_budget_config_t *config)
{
    return lp_nonnegative(config->fallback_limit_w)
        && lp_nonnegative(config->margin_w)
        && lp_nonnegative(config->charge_reserve_w)
        && lp_nonnegative(config->buffer_target_j)
        && lp_nonnegative(config->buffer_gain_w_per_j)
        && lp_nonnegative(config->cap_reserve_j)
        && lp_nonnegative(config->boost_power_w)
        && isfinite(config->boost_horizon_s) && config->boost_horizon_s > 0.0f
        && lp_nonnegative(config->absolute_max_w);
}

uint32_t lp_compute_budget(const lp_budget_config_t *config,
                          const lp_budget_input_t *input, lp_budget_t *out)
{
    uint32_t flags = LP_STATUS_OK;
    int judge_valid;
    int buffer_valid;
    int cap_valid;
    double limit;
    double buffer;
    double deficit;
    double base;
    double boost = 0.0;
    double absolute_max;
    lp_mode_t mode;

    if (out == NULL) {
        return LP_STATUS_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (config == NULL || input == NULL || !lp_budget_config_valid(config)
        || (input->requested_mode != LP_MODE_NORMAL
            && input->requested_mode != LP_MODE_CHARGE
            && input->requested_mode != LP_MODE_BOOST)) {
        out->flags = LP_STATUS_INVALID_ARGUMENT;
        return out->flags;
    }
    judge_valid = input->judge_valid && lp_nonnegative(input->judge_limit_w);
    buffer_valid = input->buffer_valid && lp_nonnegative(input->buffer_energy_j);
    cap_valid = input->cap_valid && lp_nonnegative(input->cap_energy_j);
    if (!judge_valid) {
        flags |= LP_STATUS_JUDGE_STALE;
    }
    if (!buffer_valid) {
        flags |= LP_STATUS_BUFFER_STALE;
    }
    if (!cap_valid) {
        flags |= LP_STATUS_CAP_STALE;
    }
    mode = input->requested_mode;
    if (mode == LP_MODE_BOOST && (!judge_valid || !buffer_valid || !cap_valid)) {
        mode = LP_MODE_NORMAL;
        flags |= LP_STATUS_MODE_DEGRADED;
    }
    limit = judge_valid ? (double)input->judge_limit_w
                        : (double)config->fallback_limit_w;
    buffer = buffer_valid ? (double)input->buffer_energy_j : 0.0;
    deficit = fmax(0.0, (double)config->buffer_target_j - buffer);
    base = fmax(0.0, limit - (double)config->margin_w
                    - (double)config->buffer_gain_w_per_j * deficit);
    if (mode == LP_MODE_CHARGE) {
        base = fmax(0.0, base - (double)config->charge_reserve_w);
    }
    absolute_max = (double)config->absolute_max_w;
    base = fmin(base, absolute_max);
    if (mode == LP_MODE_BOOST) {
        double energy = fmax(0.0, (double)input->cap_energy_j
                                 - (double)config->cap_reserve_j);
        boost = fmin((double)config->boost_power_w,
                     energy / (double)config->boost_horizon_s);
        boost = fmin(boost, fmax(0.0, absolute_max - base));
    }
    out->base_budget_w = lp_budget_float(base);
    out->boost_budget_w = lp_budget_float(boost);
    out->budget_w = lp_budget_float(fmin(absolute_max,
        (double)out->base_budget_w + (double)out->boost_budget_w));
    out->effective_mode = mode;
    out->flags = flags;
    return flags;
}
