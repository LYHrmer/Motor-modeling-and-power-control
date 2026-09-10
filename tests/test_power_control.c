#include "ly_power/power_control.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static unsigned checks;
static unsigned failures;

/* Unlike assert(), these checks remain active when NDEBUG is defined. */
#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        ++failures; \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    } \
} while (0)

static int near(double actual, double expected, double tolerance)
{
    return isfinite(actual) && isfinite(expected) &&
           fabs(actual - expected) <= tolerance;
}

static lp_motor_model_t copper_model(float limit)
{
    lp_motor_model_t model = {0.0f, 0.0f, 0.0f, 0.01f, 0.0f, limit};
    return model;
}

static double clipped(float request, float limit)
{
    if (request > limit) return limit;
    if (request < -limit) return -(double)limit;
    return request;
}

/* Independent double-precision model; never calls a production helper. */
static double motor_power(const lp_motor_model_t *m, double rpm, double current)
{
    return (double)m->k_mech * current * rpm +
           (double)m->k_speed_abs * fabs(rpm) +
           (double)m->k_speed_sq * rpm * rpm +
           (double)m->k_current_sq * current * current + m->p_idle;
}

static double ray_power(const lp_motor_model_t *m, const float *rpm,
                        const float *request, size_t count, double scale)
{
    double total = 0.0;
    size_t i;
    for (i = 0; i < count; ++i) {
        double p = motor_power(&m[i], rpm[i],
                               clipped(request[i], m[i].current_limit) * scale);
        if (p > 0.0) total += p;
    }
    return total;
}

/* Convex minimum by ternary search, then the rightmost feasible boundary. */
static int oracle_scale(const lp_motor_model_t *m, const float *rpm,
                        const float *request, size_t count, double budget,
                        double *scale)
{
    double left = 0.0, right = 1.0, minimum, value;
    unsigned i;
    if (ray_power(m, rpm, request, count, 1.0) <= budget) {
        *scale = 1.0;
        return 1;
    }
    for (i = 0; i < 120; ++i) {
        double a = left + (right - left) / 3.0;
        double b = right - (right - left) / 3.0;
        if (ray_power(m, rpm, request, count, a) <
            ray_power(m, rpm, request, count, b)) right = b;
        else left = a;
    }
    minimum = (left + right) / 2.0;
    value = ray_power(m, rpm, request, count, minimum);
    if (ray_power(m, rpm, request, count, 0.0) < value) {
        minimum = 0.0;
        value = ray_power(m, rpm, request, count, 0.0);
    }
    if (value > budget) return 0;
    left = minimum;
    right = 1.0;
    for (i = 0; i < 100; ++i) {
        double middle = (left + right) / 2.0;
        if (ray_power(m, rpm, request, count, middle) <= budget) left = middle;
        else right = middle;
    }
    *scale = left;
    return 1;
}

static void finite_allocation(const lp_allocation_t *out)
{
    size_t i;
    CHECK(isfinite(out->scale));
    CHECK(isfinite(out->requested_power_w));
    CHECK(isfinite(out->predicted_power_w));
    for (i = 0; i < LP_MAX_MOTORS; ++i)
        CHECK(isfinite(out->allocated_current[i]));
}

static void check_allocation(const lp_motor_model_t *m, const float *rpm,
                             const float *request, size_t count, float budget,
                             const lp_allocation_t *out)
{
    double actual_power = 0.0;
    double requested_power = ray_power(m, rpm, request, count, 1.0);
    size_t i;
    finite_allocation(out);
    CHECK(out->count == count);
    CHECK(!(out->flags & (LP_STATUS_INVALID_ARGUMENT | LP_STATUS_INVALID_MODEL |
                          LP_STATUS_NUMERIC_ERROR)));
    CHECK(out->scale >= 0.0f && out->scale <= 1.0f);
    for (i = 0; i < count; ++i) {
        double command = clipped(request[i], m[i].current_limit);
        double current = out->allocated_current[i];
        double p = motor_power(&m[i], rpm[i], current);
        CHECK(near(current, command * out->scale,
                   0.001 + 0.000005 * fabs(command)));
        CHECK(current * request[i] >= 0.0);
        CHECK(fabs(current) <= fabs(command) + 0.001);
        if (p > 0.0) actual_power += p;
    }
    CHECK(near(out->requested_power_w, requested_power,
               0.0002 + 0.000005 * requested_power));
    CHECK(near(out->predicted_power_w, actual_power,
               0.0002 + 0.000005 * actual_power));
    if (!(out->flags & LP_STATUS_INFEASIBLE))
        CHECK(actual_power <= fmax(0.0, budget) + 0.001 + 0.00002 * fabs(budget));
}

static void test_known_allocations(void)
{
    lp_motor_model_t m[4];
    float rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float request[4] = {100.0f, 0.0f, 0.0f, 0.0f};
    lp_allocation_t out;
    uint32_t flags;
    size_t i;
    m[0] = copper_model(100.0f);
    flags = lp_allocate(m, rpm, request, 1, 25.0f, &out);
    CHECK(flags == out.flags);
    CHECK(flags & LP_STATUS_POWER_LIMITED);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(near(out.scale, 0.5, 0.00002));
    CHECK(near(out.allocated_current[0], 50.0, 0.002));
    check_allocation(m, rpm, request, 1, 25.0f, &out);

    flags = lp_allocate(m, rpm, request, 1, 0.0f, &out);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(out.allocated_current[0] == 0.0f);
    request[0] = 0.0f;
    flags = lp_allocate(m, rpm, request, 1, 0.0f, &out);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(out.predicted_power_w == 0.0f);
    CHECK(near(out.scale, 1.0, 0.00001));

    /* Negative mechanical power on wheel two cannot pay for wheel one. */
    m[0].k_mech = 0.01f;
    m[0].k_current_sq = 0.0f;
    m[1] = m[0];
    rpm[0] = rpm[1] = 100.0f;
    request[0] = 100.0f;
    request[1] = -100.0f;
    flags = lp_allocate(m, rpm, request, 2, 50.0f, &out);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(near(out.scale, 0.5, 0.00002));
    CHECK(near(out.predicted_power_w, 50.0, 0.002));
    check_allocation(m, rpm, request, 2, 50.0f, &out);

    for (i = 0; i < 4; ++i) { m[i] = copper_model(100.0f); rpm[i] = 0.0f; }
    m[1].current_limit = 80.0f;
    m[2].current_limit = 200.0f;
    m[3].current_limit = 25.0f;
    request[0] = 150.0f; request[1] = -100.0f;
    request[2] = 300.0f; request[3] = -50.0f;
    flags = lp_allocate(m, rpm, request, 4, 30.0f, &out);
    CHECK(flags & LP_STATUS_CURRENT_CLAMPED);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(near(out.scale, sqrt(30.0 / 570.25), 0.00002));
    check_allocation(m, rpm, request, 4, 30.0f, &out);

    m[0] = copper_model(100.0f);
    m[0].k_speed_abs = 0.1f; m[0].p_idle = 5.0f;
    rpm[0] = 100.0f; request[0] = 50.0f;
    flags = lp_allocate(m, rpm, request, 1, 10.0f, &out);
    CHECK(flags & LP_STATUS_INFEASIBLE);
    check_allocation(m, rpm, request, 1, 10.0f, &out);
    request[0] = 0.0f;
    CHECK(lp_allocate(m, rpm, request, 1, 0.0f, &out) & LP_STATUS_INFEASIBLE);

    /* P(s)=10*s*s-10*s+2: both endpoints fail, the middle is feasible. */
    m[0] = copper_model(10.0f);
    m[0].k_mech = 0.1f; m[0].k_current_sq = 0.1f; m[0].p_idle = 2.0f;
    rpm[0] = 10.0f; request[0] = -10.0f;
    flags = lp_allocate(m, rpm, request, 1, 0.5f, &out);
    CHECK(!(flags & LP_STATUS_INFEASIBLE));
    CHECK(near(out.scale, (10.0 + sqrt(40.0)) / 20.0, 0.00003));
    check_allocation(m, rpm, request, 1, 0.5f, &out);
    m[0].p_idle = 3.0f;
    CHECK(lp_allocate(m, rpm, request, 1, 0.25f, &out) & LP_STATUS_INFEASIBLE);
}

static void test_invalid_allocations(void)
{
    lp_motor_model_t m = copper_model(100.0f);
    lp_allocation_t out;
    float rpm = 0.0f, request = 100.0f;
    const float bad[] = {NAN, INFINITY, -INFINITY};
    size_t i, j;
    uint32_t flags;
    CHECK(lp_allocate(NULL, &rpm, &request, 1, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_allocate(&m, NULL, &request, 1, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_allocate(&m, &rpm, NULL, 1, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_allocate(&m, &rpm, &request, 1, 1.0f, NULL) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_allocate(&m, &rpm, &request, 0, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_allocate(&m, &rpm, &request, LP_MAX_MOTORS + 1u, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
    for (i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        CHECK(lp_allocate(&m, &bad[i], &request, 1, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
        finite_allocation(&out);
        CHECK(lp_allocate(&m, &rpm, &bad[i], 1, 1.0f, &out) & LP_STATUS_INVALID_ARGUMENT);
        CHECK(lp_allocate(&m, &rpm, &request, 1, bad[i], &out) & LP_STATUS_INVALID_ARGUMENT);
    }
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 3; ++j) {
            lp_motor_model_t invalid = m;
            float *field[] = {&invalid.k_mech, &invalid.k_speed_abs,
                &invalid.k_speed_sq, &invalid.k_current_sq, &invalid.p_idle,
                &invalid.current_limit};
            *field[i] = j == 0 ? -1.0f : bad[j - 1];
            CHECK(lp_allocate(&invalid, &rpm, &request, 1, 1.0f, &out) & LP_STATUS_INVALID_MODEL);
            finite_allocation(&out);
        }
    }
    m.current_limit = 0.0f;
    CHECK(lp_allocate(&m, &rpm, &request, 1, 1.0f, &out) & LP_STATUS_INVALID_MODEL);
    m = copper_model(100.0f);
    flags = lp_allocate(&m, &rpm, &request, 1, -5.0f, &out);
    CHECK(flags & LP_STATUS_NEGATIVE_BUDGET);
    CHECK(!(flags & LP_STATUS_INVALID_ARGUMENT));
    CHECK(out.allocated_current[0] == 0.0f);
    m.k_current_sq = FLT_MAX; m.current_limit = FLT_MAX; request = FLT_MAX;
    flags = lp_allocate(&m, &rpm, &request, 1, FLT_MAX, &out);
    CHECK(flags & LP_STATUS_NUMERIC_ERROR);
    finite_allocation(&out);
}

static uint32_t random_state = UINT32_C(0x4d4f544f);
static float random_between(float low, float high)
{
    random_state = random_state * UINT32_C(1664525) + UINT32_C(1013904223);
    return low + (high - low) * (float)(random_state >> 8) / 16777216.0f;
}

static void test_random_allocations(void)
{
    unsigned trial;
    for (trial = 0; trial < 400; ++trial) {
        lp_motor_model_t m[LP_MAX_MOTORS];
        float rpm[LP_MAX_MOTORS], request[LP_MAX_MOTORS];
        float budget = trial % 17 == 0 ? 0.0f : random_between(0.0f, 250.0f);
        size_t count = 1u + trial % LP_MAX_MOTORS, i;
        double expected_scale = 0.0;
        lp_allocation_t out;
        int feasible;
        uint32_t flags;
        for (i = 0; i < count; ++i) {
            m[i].current_limit = random_between(100.0f, 5000.0f);
            m[i].k_mech = trial % 9 == 0 ? 0.0f : random_between(0.0f, 0.0001f);
            m[i].k_speed_abs = random_between(0.0f, 0.002f);
            m[i].k_speed_sq = random_between(0.0f, 0.000002f);
            m[i].k_current_sq = trial % 7 == 0 ? 0.0f : random_between(0.0f, 0.000006f);
            m[i].p_idle = random_between(0.0f, 4.0f);
            rpm[i] = random_between(-1500.0f, 1500.0f);
            request[i] = random_between(-8000.0f, 8000.0f);
        }
        flags = lp_allocate(m, rpm, request, count, budget, &out);
        CHECK(flags == out.flags);
        check_allocation(m, rpm, request, count, budget, &out);
        feasible = oracle_scale(m, rpm, request, count, budget, &expected_scale);
        CHECK(feasible == !(flags & LP_STATUS_INFEASIBLE));
        if (feasible) CHECK(near(out.scale, expected_scale, 0.0004));
    }
}

static void test_budgets(void)
{
    lp_budget_config_t cfg = {40.0f, 5.0f, 10.0f, 20.0f, 2.0f,
                              30.0f, 50.0f, 2.0f, 200.0f};
    lp_budget_input_t in = {80.0f, 15.0f, 70.0f, 1, 1, 1, LP_MODE_NORMAL};
    lp_budget_t out;
    const float bad[] = {-1.0f, NAN, INFINITY};
    const uint32_t stale[] = {LP_STATUS_JUDGE_STALE, LP_STATUS_BUFFER_STALE,
                              LP_STATUS_CAP_STALE};
    const float fallback[] = {25.0f, 35.0f, 65.0f};
    size_t i, j;
    uint32_t flags = lp_compute_budget(&cfg, &in, &out);
    CHECK(flags == LP_STATUS_OK && out.flags == flags);
    CHECK(out.effective_mode == LP_MODE_NORMAL);
    CHECK(near(out.base_budget_w, 65.0, 0.0001));
    CHECK(out.boost_budget_w == 0.0f);
    CHECK(near(out.budget_w, 65.0, 0.0001));
    in.requested_mode = LP_MODE_CHARGE;
    CHECK(lp_compute_budget(&cfg, &in, &out) == LP_STATUS_OK);
    CHECK(out.effective_mode == LP_MODE_CHARGE);
    CHECK(near(out.budget_w, 55.0, 0.0001));
    in.requested_mode = LP_MODE_BOOST;
    CHECK(lp_compute_budget(&cfg, &in, &out) == LP_STATUS_OK);
    CHECK(near(out.boost_budget_w, 20.0, 0.0001));
    CHECK(near(out.budget_w, 85.0, 0.0001));
    CHECK(out.boost_budget_w * cfg.boost_horizon_s <= in.cap_energy_j - cfg.cap_reserve_j);
    in.cap_energy_j = 230.0f;
    CHECK(lp_compute_budget(&cfg, &in, &out) == LP_STATUS_OK);
    CHECK(near(out.boost_budget_w, 50.0, 0.0001));
    in.cap_energy_j = 70.0f; cfg.absolute_max_w = 75.0f;
    CHECK(lp_compute_budget(&cfg, &in, &out) == LP_STATUS_OK);
    CHECK(near(out.base_budget_w, 65.0, 0.0001));
    CHECK(near(out.boost_budget_w, 10.0, 0.0001));
    CHECK(near(out.budget_w, 75.0, 0.0001));
    cfg.absolute_max_w = 200.0f;
    for (i = 0; i < 3; ++i) {
        lp_budget_input_t invalid = in;
        int *valid[] = {&invalid.judge_valid, &invalid.buffer_valid, &invalid.cap_valid};
        *valid[i] = 0;
        flags = lp_compute_budget(&cfg, &invalid, &out);
        CHECK(flags == out.flags);
        CHECK(flags & stale[i]);
        CHECK(flags & LP_STATUS_MODE_DEGRADED);
        CHECK(out.effective_mode == LP_MODE_NORMAL);
        CHECK(out.boost_budget_w == 0.0f);
        CHECK(near(out.budget_w, fallback[i], 0.0001));
        for (j = 0; j < 3; ++j) {
            lp_budget_input_t malformed = in;
            float *field[] = {&malformed.judge_limit_w, &malformed.buffer_energy_j,
                              &malformed.cap_energy_j};
            *field[i] = bad[j];
            flags = lp_compute_budget(&cfg, &malformed, &out);
            CHECK(flags & stale[i]);
            CHECK(out.effective_mode == LP_MODE_NORMAL);
            CHECK(out.boost_budget_w == 0.0f && isfinite(out.budget_w));
        }
    }
    in.cap_energy_j = cfg.cap_reserve_j;
    (void)lp_compute_budget(&cfg, &in, &out);
    CHECK(out.boost_budget_w == 0.0f);
    CHECK(lp_compute_budget(NULL, &in, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_compute_budget(&cfg, NULL, &out) & LP_STATUS_INVALID_ARGUMENT);
    CHECK(lp_compute_budget(&cfg, &in, NULL) & LP_STATUS_INVALID_ARGUMENT);
    in.requested_mode = (lp_mode_t)99;
    CHECK(lp_compute_budget(&cfg, &in, &out) & LP_STATUS_INVALID_ARGUMENT);
    in.requested_mode = LP_MODE_NORMAL;
    for (i = 0; i < 9; ++i) {
        for (j = 0; j < 3; ++j) {
            lp_budget_config_t invalid = cfg;
            float *field[] = {&invalid.fallback_limit_w, &invalid.margin_w,
                &invalid.charge_reserve_w, &invalid.buffer_target_j,
                &invalid.buffer_gain_w_per_j, &invalid.cap_reserve_j,
                &invalid.boost_power_w, &invalid.boost_horizon_s, &invalid.absolute_max_w};
            *field[i] = bad[j];
            CHECK(lp_compute_budget(&invalid, &in, &out) & LP_STATUS_INVALID_ARGUMENT);
            CHECK(isfinite(out.budget_w));
        }
    }
    cfg.boost_horizon_s = 0.0f;
    CHECK(lp_compute_budget(&cfg, &in, &out) & LP_STATUS_INVALID_ARGUMENT);
    cfg.boost_horizon_s = 2.0f; cfg.absolute_max_w = 0.0f;
    CHECK(lp_compute_budget(&cfg, &in, &out) == LP_STATUS_OK);
    CHECK(out.budget_w == 0.0f);
}

int main(void)
{
    test_known_allocations();
    test_invalid_allocations();
    test_random_allocations();
    test_budgets();
    printf("power control: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
