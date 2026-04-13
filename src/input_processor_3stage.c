/*
 * Copyright (c) 2025 soranomutech
 * SPDX-License-Identifier: MIT
 *
 * 3-Stage Pointing Acceleration Input Processor for ZMK
 *
 * Implements up to a 5-zone acceleration curve:
 *   Zone A (0 → speed_flat_low):       flat at min_factor           (optional)
 *   Zone B (speed_flat_low → speed_low): min_factor → mid_factor
 *   Zone C (speed_low → speed_high):   flat at mid_factor
 *   Zone D (speed_high → speed_max):   mid_factor → max_factor
 *   Zone E (speed_max →):              flat at max_factor
 *
 * If speed_flat_low == 0, Zone A is disabled and the curve behaves as the
 * original 3-zone (decel/flat/accel) configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#define DT_DRV_COMPAT zmk_input_processor_3stage

#define SCALE 1000

/* ── Configuration from devicetree ──────────────────────────── */

struct accel_3s_config {
    uint8_t  input_type;
    bool     track_remainders;
    uint16_t min_factor;       /* factor at speed=0 (x1000)         */
    uint16_t mid_factor;       /* factor in flat zone (x1000)       */
    uint16_t max_factor;       /* factor at speed_max (x1000)       */
    uint32_t speed_flat_low;   /* end of low flat zone (cps, 0=off) */
    uint32_t speed_low;        /* end of deceleration zone (cps)    */
    uint32_t speed_high;       /* start of acceleration zone (cps)  */
    uint32_t speed_max;        /* speed at which max_factor reached */
    uint8_t  decel_exponent;   /* curve for decel zone (1=linear)   */
    uint8_t  accel_exponent;   /* curve for accel zone (2=quadratic)*/
};

/* ── Per-instance runtime data ──────────────────────────────── */

struct accel_3s_data {
    int64_t last_time_ms[2];   /* per-axis timestamp (X=0, Y=1)     */
    int16_t remainders[2];     /* fractional remainder (x1000)       */
    int8_t  last_sign[2];      /* last movement direction per axis   */
};

/* ── Integer power: (t/1000)^exp * 1000 ─────────────────────── */

static uint32_t pow_scaled(uint32_t t, uint8_t exp) {
    if (t > SCALE) {
        t = SCALE;
    }
    if (exp <= 1) {
        return t;
    }
    uint64_t acc = t;
    for (uint8_t i = 1; i < exp; i++) {
        acc = (acc * t) / SCALE;
    }
    return (acc > UINT32_MAX) ? UINT32_MAX : (uint32_t)acc;
}

/* ── 3-stage factor computation ─────────────────────────────── */

static uint32_t compute_factor(const struct accel_3s_config *cfg, uint32_t cps) {
    /* Zone A: Low flat (0 → speed_flat_low) */
    if (cfg->speed_flat_low > 0 && cps < cfg->speed_flat_low) {
        return cfg->min_factor;
    }

    /* Zone B: Deceleration / ramp (speed_flat_low → speed_low) */
    if (cfg->speed_low > cfg->speed_flat_low && cps < cfg->speed_low) {
        uint32_t range = cfg->speed_low - cfg->speed_flat_low;
        uint32_t t = (uint32_t)((uint64_t)(cps - cfg->speed_flat_low) * SCALE / range);
        uint32_t shaped = pow_scaled(t, cfg->decel_exponent);
        int32_t span = (int32_t)cfg->mid_factor - (int32_t)cfg->min_factor;
        return (uint32_t)((int32_t)cfg->min_factor +
                          (int32_t)((int64_t)span * shaped / SCALE));
    }

    /* Zone C: Mid flat (speed_low → speed_high) */
    if (cps <= cfg->speed_high) {
        return cfg->mid_factor;
    }

    /* Zone D: Acceleration (speed_high → speed_max) */
    if (cps >= cfg->speed_max) {
        return cfg->max_factor;
    }

    uint32_t range = cfg->speed_max - cfg->speed_high;
    if (range == 0) {
        return cfg->max_factor;
    }

    uint32_t t = (uint32_t)((uint64_t)(cps - cfg->speed_high) * SCALE / range);
    uint32_t shaped = pow_scaled(t, cfg->accel_exponent);
    int32_t span = (int32_t)cfg->max_factor - (int32_t)cfg->mid_factor;
    return (uint32_t)((int32_t)cfg->mid_factor +
                      (int32_t)((int64_t)span * shaped / SCALE));
}

/* ── Event handler ──────────────────────────────────────────── */

static int accel_3s_handle_event(const struct device *dev,
                                 struct input_event *event,
                                 uint32_t param1, uint32_t param2,
                                 struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct accel_3s_config *cfg = dev->config;
    struct accel_3s_data *data = dev->data;

    if (event->type != cfg->input_type) {
        return 0;
    }

    /* Determine axis index */
    uint32_t idx;
    if (event->code == INPUT_REL_X) {
        idx = 0;
    } else if (event->code == INPUT_REL_Y) {
        idx = 1;
    } else {
        return 0;
    }

    const int32_t raw = event->value;
    const int64_t now = k_uptime_get();

    if (raw == 0) {
        data->last_time_ms[idx] = now;
        return 0;
    }

    /* Reset remainder on direction change */
    int8_t sign = (raw > 0) ? 1 : -1;
    if (cfg->track_remainders && data->last_sign[idx] != 0 &&
        data->last_sign[idx] != sign) {
        data->remainders[idx] = 0;
    }
    data->last_sign[idx] = sign;

    /* Compute velocity in counts per second */
    uint32_t factor;
    if (data->last_time_ms[idx] == 0 || now <= data->last_time_ms[idx] ||
        (now - data->last_time_ms[idx]) > 150) {
        /* First event or returning from idle: use mid-factor (no decel/accel) */
        factor = cfg->mid_factor;
    } else {
        uint32_t dt_ms = (uint32_t)(now - data->last_time_ms[idx]);
        if (dt_ms == 0) {
            dt_ms = 1;
        }
        uint32_t cps = (uint32_t)((uint64_t)abs(raw) * 1000ULL / dt_ms);
        factor = compute_factor(cfg, cps);
    }

    /* Apply factor to raw input */
    if (cfg->track_remainders) {
        int64_t total = (int64_t)raw * (int64_t)factor
                      + (int64_t)data->remainders[idx];
        int32_t out = (int32_t)(total / SCALE);
        int32_t rem = (int32_t)(total - (int64_t)out * SCALE);

        if (out > 32767)  out = 32767;
        if (out < -32768) out = -32768;

        event->value = out;
        data->remainders[idx] = (int16_t)rem;
    } else {
        int32_t out = (int32_t)(((int64_t)raw * (int64_t)factor) / SCALE);

        /* Guarantee minimum 1 unit when raw != 0 */
        if (out == 0) {
            out = (raw > 0) ? 1 : -1;
        }

        event->value = out;
    }

    data->last_time_ms[idx] = now;
    return 0;
}

/* ── ZMK driver API ─────────────────────────────────────────── */

static const struct zmk_input_processor_driver_api accel_3s_api = {
    .handle_event = accel_3s_handle_event,
};

/* ── Device instantiation from devicetree ───────────────────── */

#define ACCEL_3S_INST(inst)                                                      \
    static const struct accel_3s_config accel_3s_config_##inst = {               \
        .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),           \
        .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),       \
        .min_factor = DT_INST_PROP(inst, min_factor),                            \
        .mid_factor = DT_INST_PROP(inst, mid_factor),                            \
        .max_factor = DT_INST_PROP(inst, max_factor),                            \
        .speed_flat_low = DT_INST_PROP_OR(inst, speed_flat_low, 0),              \
        .speed_low = DT_INST_PROP(inst, speed_low),                              \
        .speed_high = DT_INST_PROP(inst, speed_high),                            \
        .speed_max = DT_INST_PROP(inst, speed_max),                              \
        .decel_exponent = DT_INST_PROP_OR(inst, deceleration_exponent, 1),       \
        .accel_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 2),       \
    };                                                                           \
    static struct accel_3s_data accel_3s_data_##inst = {0};                      \
    DEVICE_DT_INST_DEFINE(inst,                                                  \
                          NULL,                                                  \
                          NULL,                                                  \
                          &accel_3s_data_##inst,                                 \
                          &accel_3s_config_##inst,                               \
                          POST_KERNEL,                                           \
                          50,                                                    \
                          &accel_3s_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_3S_INST)
