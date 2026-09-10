/* Copyright (c) 2026 LYHrmer. SPDX-License-Identifier: MIT
 * See the repository license.txt for the license text.
 */
#ifndef LY_POWER_POWER_CONTROL_H
#define LY_POWER_POWER_CONTROL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LP_MAX_MOTORS 8u

#define LP_STATUS_OK               UINT32_C(0)
#define LP_STATUS_INVALID_ARGUMENT (UINT32_C(1) << 0)
#define LP_STATUS_INVALID_MODEL    (UINT32_C(1) << 1)
#define LP_STATUS_NUMERIC_ERROR    (UINT32_C(1) << 2)
#define LP_STATUS_CURRENT_CLAMPED  (UINT32_C(1) << 3)
#define LP_STATUS_POWER_LIMITED    (UINT32_C(1) << 4)
#define LP_STATUS_INFEASIBLE       (UINT32_C(1) << 5)
#define LP_STATUS_NEGATIVE_BUDGET  (UINT32_C(1) << 6)
#define LP_STATUS_JUDGE_STALE      (UINT32_C(1) << 7)
#define LP_STATUS_BUFFER_STALE     (UINT32_C(1) << 8)
#define LP_STATUS_CAP_STALE        (UINT32_C(1) << 9)
#define LP_STATUS_MODE_DEGRADED    (UINT32_C(1) << 10)

/* Per-motor model, fitted for the actual motor, ESC, and measurement boundary.
 * u is signed DJI raw current command units, NOT amperes. Speed is signed rpm
 * at the shaft used during fitting; rotor and output-shaft rpm cannot be mixed.
 * P(W) = k_mech*u*rpm + k_speed_abs*abs(rpm) + k_speed_sq*rpm*rpm
 *      + k_current_sq*u*u + p_idle.
 * All coefficients must be finite and nonnegative; current_limit must be
 * finite and positive. No sample coefficient is a hardware calibration.
 */
typedef struct {
    float k_mech;
    float k_speed_abs;
    float k_speed_sq;
    float k_current_sq;
    float p_idle;
    float current_limit;
} lp_motor_model_t;

typedef struct {
    uint32_t flags;
    size_t count;
    float scale;
    /* Conservative sum(max(P_i, 0)) at s=1 AFTER per-motor current clamps. */
    float requested_power_w;
    /* Recomputed from the actual returned float current commands. */
    float predicted_power_w;
    /* u_i = float(scale * clamp(request_i, +/- current_limit_i)).
     * Signs and ratios of clamped requests are preserved to float precision.
     * These values are not yet rounded to the ESC's integer wire format.
     */
    float allocated_current[LP_MAX_MOTORS];
} lp_allocation_t;

/* Largest feasible common scale on [0,1], using conservative positive motor
 * powers: regeneration never pays for another motor's consumption.
 * Braking can make P(s) nonmonotonic. The convex minimum is found first, then
 * the right-hand feasible boundary. If no feasible output exists, INFEASIBLE
 * is set and the lowest predicted-consumption command on this scaling ray is
 * returned. Such a command is NOT a satisfied power limit or a hardware stop.
 * A negative budget is treated as zero with NEGATIVE_BUDGET; zero is valid.
 * Invalid arguments/models produce zero-initialized output, count=0, and an
 * error flag. out may not alias input storage. A null out returns an error.
 * Requested or predicted powers exceeding FLT_MAX saturate to FLT_MAX and set
 * NUMERIC_ERROR; callers must not interpret saturated power as a measurement.
 * A representability failure at a feasible real-valued minimum also sets
 * NUMERIC_ERROR and INFEASIBLE. All returned floats remain finite.
 * This stateless routine allocates no memory. It uses double intermediates;
 * profile worst-case execution time on MCUs without double-precision FPUs.
 */
uint32_t lp_allocate(const lp_motor_model_t *models, const float *rpm,
                     const float *requested_current, size_t count,
                     float budget_w, lp_allocation_t *out);

typedef enum {
    LP_MODE_NORMAL = 0,
    LP_MODE_CHARGE = 1,
    LP_MODE_BOOST = 2
} lp_mode_t;

/* All fields must be finite and nonnegative; boost_horizon_s must be positive.
 * These are integration settings, not an interpretation of competition rules.
 */
typedef struct {
    float fallback_limit_w;
    float margin_w;
    float charge_reserve_w;
    float buffer_target_j;
    float buffer_gain_w_per_j;
    float cap_reserve_j;
    float boost_power_w;
    float boost_horizon_s;
    float absolute_max_w;
} lp_budget_config_t;

typedef struct {
    float judge_limit_w;
    float buffer_energy_j;
    /* Usable energy measured above the hardware's minimum operating voltage,
     * e.g. 0.5*C*(V*V - V_min*V_min); cap_reserve_j is an additional reserve.
     */
    float cap_energy_j;
    /* Nonzero means recent, trusted data. Invalid numeric readings are stale
     * even if the corresponding valid flag is nonzero.
     */
    int judge_valid;
    int buffer_valid;
    int cap_valid;
    lp_mode_t requested_mode;
} lp_budget_input_t;

typedef struct {
    uint32_t flags;
    lp_mode_t effective_mode;
    float base_budget_w;
    float boost_budget_w;
    float budget_w;
} lp_budget_t;

/* Base = max(0, limit - margin - gain*max(0, buffer_target - buffer)).
 * The configured fallback replaces a stale judge limit. A stale buffer is
 * treated as zero available energy. CHARGE additionally subtracts its reserve.
 * BOOST adds min(boost_power, max(0, cap_energy-cap_reserve)/boost_horizon).
 * Total power is capped by absolute_max. Any stale input disables BOOST and
 * reports MODE_DEGRADED; individual stale flags are reported in every mode.
 * Returned budgets are rounded downward, not above the computed constraints.
 * BOOST is a downstream motor budget and may exceed the judge input limit;
 * the integrator must enforce source-side limits in the capacitor hardware.
 * No clock, RTOS, or energy state is maintained here. The caller controls dt,
 * validates timestamps, refreshes energy feedback, and chooses a horizon that
 * is at least the interval until the next trusted update. Reusing old energy
 * with valid=1 does not enforce an energy limit over time.
 */
uint32_t lp_compute_budget(const lp_budget_config_t *config,
                          const lp_budget_input_t *input, lp_budget_t *out);

#ifdef __cplusplus
}
#endif

#endif
