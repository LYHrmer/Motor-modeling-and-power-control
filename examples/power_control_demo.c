/* Copyright (c) 2026 LYHrmer. SPDX-License-Identifier: MIT
 * See the repository license.txt for the license text.
 * Synthetic coefficients only: this is not a motor or capacitor calibration.
 */
#include "ly_power/power_control.h"

#include <stdio.h>

static void print_allocation(const char *label, float budget,
                             const lp_allocation_t *allocation)
{
    size_t i;
    printf("%-11s budget=%7.3f W requested=%7.3f W predicted=%7.3f W "
           "scale=%.6f flags=0x%03lx current=[",
           label, (double)budget, (double)allocation->requested_power_w,
           (double)allocation->predicted_power_w, (double)allocation->scale,
           (unsigned long)allocation->flags);
    for (i = 0; i < allocation->count; ++i) {
        printf("%s%.3f", i == 0u ? "" : ", ",
               (double)allocation->allocated_current[i]);
    }
    puts("] DJI raw units");
}

int main(void)
{
    lp_motor_model_t models[4];
    const float speed[4] = {1500.0f, 1600.0f, 1400.0f, 1550.0f};
    const float requested[4] = {5000.0f, 4800.0f, 5100.0f, 4900.0f};
    const lp_budget_config_t config = {
        40.0f, 5.0f, 15.0f, 40.0f, 1.0f, 30.0f, 50.0f, 2.0f, 150.0f
    };
    lp_budget_input_t input = {80.0f, 40.0f, 130.0f, 1, 1, 1, LP_MODE_NORMAL};
    const char *labels[3] = {"NORMAL", "CHARGE", "BOOST"};
    lp_budget_t budget;
    lp_allocation_t allocation;
    size_t i;
    int mode;

    puts("Synthetic model demonstration; no hardware measurements claimed.");
    for (i = 0; i < 4u; ++i) {
        models[i].k_mech = 3.0e-6f;
        models[i].k_speed_abs = 0.001f;
        models[i].k_speed_sq = 1.0e-7f;
        models[i].k_current_sq = 1.0e-7f;
        models[i].p_idle = 1.0f;
        models[i].current_limit = 10000.0f;
    }
    for (mode = LP_MODE_NORMAL; mode <= LP_MODE_BOOST; ++mode) {
        input.requested_mode = (lp_mode_t)mode;
        (void)lp_compute_budget(&config, &input, &budget);
        (void)lp_allocate(models, speed, requested, 4u, budget.budget_w,
                          &allocation);
        print_allocation(labels[mode], budget.budget_w, &allocation);
    }
    {
        /* Each wheel: P(s)=max(10*s*s-10*s+2,0). At s=0 total
         * power is 8 W, yet an intermediate braking command fits 2 W.
         */
        const float braking_speed[4] = {10.0f, 10.0f, 10.0f, 10.0f};
        const float braking_request[4] = {-10.0f, -10.0f, -10.0f, -10.0f};
        for (i = 0; i < 4u; ++i) {
            models[i].k_mech = 0.1f;
            models[i].k_speed_abs = 0.0f;
            models[i].k_speed_sq = 0.0f;
            models[i].k_current_sq = 0.1f;
            models[i].p_idle = 2.0f;
            models[i].current_limit = 10.0f;
        }
        (void)lp_allocate(models, braking_speed, braking_request, 4u, 2.0f,
                          &allocation);
        print_allocation("BRAKING", 2.0f, &allocation);
        for (i = 0; i < 4u; ++i) {
            models[i].p_idle = 3.0f;
        }
        (void)lp_allocate(models, braking_speed, braking_request, 4u, 1.0f,
                          &allocation);
        print_allocation("INFEASIBLE", 1.0f, &allocation);
    }
    return 0;
}
