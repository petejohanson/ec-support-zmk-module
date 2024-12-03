/*
 * Copyright (c) 2022, 2023 Kan-Ru Chen
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include "zmk_kscan_ec_matrix.h"

#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_kscan_ec_matrix);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC) ||                                       \
    IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
#include <zephyr/timing/timing.h>
#endif

struct kscan_ec_matrix_config {
    const struct pinctrl_dev_config *pcfg;
    struct gpio_dt_spec power;
    struct gpio_dt_spec drain;
    const struct adc_dt_spec adc_channel;
    const bool skip_startup_calibration;
    const uint8_t strobes_len;
    const uint8_t inputs_len;
    const uint16_t matrix_warm_up_us;
    const uint16_t matrix_relax_us;
    const uint16_t adc_read_settle_us;
    const uint16_t active_polling_interval_ms;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    const uint16_t idle_polling_interval_ms;
    const uint16_t sleep_polling_interval_ms;
    const uint16_t idle_after_secs;
    const uint16_t sleep_after_secs;
    const bool dynamic_polling_interval;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    const struct gpio_dt_spec *inputs;
    const uint32_t *strobe_input_masks;
    const struct gpio_dt_spec strobes[];
};

struct kscan_ec_matrix_data {
    kscan_callback_t callback;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    uint32_t last_key_released_at;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    uint16_t poll_interval;
    struct k_thread thread;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_STACK_SIZE);
    const struct device *dev;
    struct k_mutex mutex;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)
    zmk_kscan_ec_matrix_calibration_cb_t calibration_callback;
    const void *calibration_user_data;
#endif // IS_DEFINED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
    uint64_t max_scan_duration_ns;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    struct zmk_kscan_ec_matrix_read_timing read_timing;
#endif
    struct zmk_kscan_ec_matrix_calibration_entry *calibrations;
    uint64_t *reported_matrix_state;
    uint64_t matrix_state[];
};

static int kscan_ec_matrix_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_ec_matrix_data *data = dev->data;
    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int kscan_ec_matrix_enable(const struct device *dev) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    data->poll_interval = cfg->active_polling_interval_ms;

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    data->last_key_released_at = k_uptime_get();
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

    k_mutex_unlock(&data->mutex);

    return 0;
}

static int kscan_ec_matrix_disable(const struct device *dev) {
    struct kscan_ec_matrix_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_MSEC(30));

    return 0;
}

struct zmk_kscan_ec_matrix_calibration_entry *
calibration_entry_for_strobe_input(const struct device *dev, uint8_t strobe, uint8_t input) {
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;

    return &data->calibrations[(strobe * cfg->inputs_len) + input];
}

static uint16_t read_raw_matrix_state(const struct device *dev, uint8_t strobe, uint8_t input) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    int ret;

    int16_t buf = 0;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    struct kscan_ec_matrix_data *data = dev->data;

    timing_start();
    timing_t start_time = timing_counter_get();
#endif

    adc_sequence_init_dt(&cfg->adc_channel, &sequence);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t adc_init_done = timing_counter_get();
#endif

    ret = gpio_pin_configure_dt(&cfg->inputs[input], GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to set the input pin (%d)", ret);
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t gpio_input_done = timing_counter_get();
#endif

    // TODO: Only wait as long as is need after drain pin was set low.
    if (cfg->matrix_relax_us > 0) {
        k_busy_wait(cfg->matrix_relax_us);
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t relax_done = timing_counter_get();
#endif

    const uint32_t lock = irq_lock();

    if (cfg->drain.port != NULL) {
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_FAKE_OPEN_DRAIN)
        gpio_pin_configure_dt(&cfg->drain, GPIO_INPUT);
#else
        gpio_pin_set_dt(&cfg->drain, 1);
#endif
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t drain_released_done = timing_counter_get();
#endif

    gpio_pin_set_dt(&cfg->strobes[strobe], 1);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t set_strobe_done = timing_counter_get();
#endif

    k_busy_wait(cfg->adc_read_settle_us);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t adc_read_settle_done = timing_counter_get();
#endif

    ret = adc_read(cfg->adc_channel.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC READ ERROR %d", ret);
    }

    irq_unlock(lock);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t adc_read_done = timing_counter_get();
#endif

    gpio_pin_set_dt(&cfg->strobes[strobe], 0);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t strobe_unset_done = timing_counter_get();
#endif

    if (cfg->drain.port != NULL) {
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_FAKE_OPEN_DRAIN)
        gpio_pin_configure_dt(&cfg->drain, GPIO_OUTPUT);
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_FAKE_OPEN_DRAIN)
        gpio_pin_set_dt(&cfg->drain, 0);
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t drain_unset_done = timing_counter_get();
#endif

    gpio_pin_configure_dt(&cfg->inputs[input], GPIO_DISCONNECTED);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    timing_t gpio_input_disconnect_done = timing_counter_get();

    timing_stop();

    data->read_timing = (struct zmk_kscan_ec_matrix_read_timing){
        .total_ns =
            timing_cycles_to_ns(timing_cycles_get(&start_time, &gpio_input_disconnect_done)),
        .adc_sequence_init_ns = timing_cycles_to_ns(timing_cycles_get(&start_time, &adc_init_done)),
        .gpio_input_ns = timing_cycles_to_ns(timing_cycles_get(&adc_init_done, &gpio_input_done)),
        .relax_ns = timing_cycles_to_ns(timing_cycles_get(&gpio_input_done, &relax_done)),
        .plug_drain_ns = timing_cycles_to_ns(timing_cycles_get(&relax_done, &drain_released_done)),
        .set_strobe_ns =
            timing_cycles_to_ns(timing_cycles_get(&drain_released_done, &set_strobe_done)),
        .read_settle_ns =
            timing_cycles_to_ns(timing_cycles_get(&set_strobe_done, &adc_read_settle_done)),
        .adc_read_ns =
            timing_cycles_to_ns(timing_cycles_get(&adc_read_settle_done, &adc_read_done)),
        .unset_strobe_ns =
            timing_cycles_to_ns(timing_cycles_get(&adc_read_done, &strobe_unset_done)),
        .pull_drain_ns =
            timing_cycles_to_ns(timing_cycles_get(&strobe_unset_done, &drain_unset_done)),
        .input_disconnect_ns =
            timing_cycles_to_ns(timing_cycles_get(&drain_unset_done, &gpio_input_disconnect_done)),
    };
#endif

    return buf;
}

#define SAMPLE_COUNT 20

struct sample_results {
    uint16_t min;
    uint16_t max;
    uint16_t avg;
    uint16_t noise;
};

struct sample_results sample(const struct device *dev, int s, int i) {
    uint16_t min = 0, max = 0, avg = 0;

    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
        uint16_t val = read_raw_matrix_state(dev, s, i);

        if (sample == 0) {
            avg = min = max = val;
        } else {
            max = MAX(val, max);
            min = MIN(val, min);
            avg = ((avg * sample) + val) / (sample + 1);
        }

        k_sleep(K_MSEC(1));
    }

    return (struct sample_results){
        .min = min,
        .max = max,
        .avg = avg,
        .noise = max - min,
    };
}

uint16_t normalize(uint16_t val, uint16_t avg_low, uint16_t avg_high) {
    val = MAX(val, avg_low);
    val = MIN(val, avg_high);

    uint32_t numerator = UINT16_MAX * (val - avg_low);
    uint16_t denominator = avg_high - avg_low;

    return (uint16_t)(numerator / denominator);
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)

void calibrate(const struct device *dev) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;
    uint16_t keys_to_complete = 0;
    if (data->calibration_callback) {
        struct zmk_kscan_ec_matrix_calibration_event ev = {
            .type = CALIBRATION_EV_LOW_SAMPLING_START, .data = {}};
        data->calibration_callback(&ev, data->calibration_user_data);
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 1);
        k_busy_wait(cfg->matrix_warm_up_us);
    }

    // Read one sample and toss it. This ensures the ADC has been enabled before taking real
    // samples.
    read_raw_matrix_state(dev, 0, 0);

    for (int s = 0; s < cfg->strobes_len; s++) {
        for (int i = 0; i < cfg->inputs_len; i++) {
            if (cfg->strobe_input_masks && (cfg->strobe_input_masks[s] & BIT(i)) != 0) {
                continue;
            }

            struct zmk_kscan_ec_matrix_calibration_entry *calibration =
                calibration_entry_for_strobe_input(dev, s, i);
            memset(calibration, 0, sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
            struct sample_results low_res = sample(dev, s, i);

            LOG_DBG("Low avg for %d,%d using %d and %d is %d. Noise %d", s, i, low_res.max,
                    low_res.min, low_res.avg, low_res.noise);
            if (data->calibration_callback) {
                struct zmk_kscan_ec_matrix_calibration_event ev = {
                    .type = CALIBRATION_EV_POSITION_LOW_DETERMINED,
                    .data = {.position_low_determined = {.low_avg = low_res.avg,
                                                         .strobe = s,
                                                         .input = i,
                                                         .noise = low_res.noise}}};
                data->calibration_callback(&ev, data->calibration_user_data);
            }

            calibration->avg_low = low_res.avg;
            calibration->noise = low_res.noise;
            keys_to_complete++;
        }
    }

    if (data->calibration_callback) {
        struct zmk_kscan_ec_matrix_calibration_event ev = {
            .type = CALIBRATION_EV_HIGH_SAMPLING_START, .data = {}};
        data->calibration_callback(&ev, data->calibration_user_data);
    }

    while (keys_to_complete > 0) {
        for (int s = 0; s < cfg->strobes_len; s++) {
            for (int i = 0; i < cfg->inputs_len; i++) {
                if (cfg->strobe_input_masks && (cfg->strobe_input_masks[s] & BIT(i)) != 0) {
                    continue;
                }

                struct zmk_kscan_ec_matrix_calibration_entry *calibration =
                    calibration_entry_for_strobe_input(dev, s, i);

                if (calibration->avg_high > 0) {
                    continue;
                }

                // Set the high threshold to half the full range possible
                uint16_t high_threshold = (1 << (cfg->adc_channel.resolution - 1));
                uint16_t high_check_val = read_raw_matrix_state(dev, s, i);

                if (high_check_val < high_threshold) {
                    continue;
                }

                k_sleep(K_MSEC(1));

                // Double checks to filter funky random one-off spikes
                high_check_val = read_raw_matrix_state(dev, s, i);

                if (high_check_val < high_threshold) {
                    continue;
                }

                LOG_WRN("Getting high for %d/%d after %d is higher than threashold: %d for "
                        "resolution %d",
                        s, i, high_check_val, high_threshold, cfg->adc_channel.resolution);
                k_sleep(K_MSEC(200));

                struct sample_results high_res = sample(dev, s, i);

                // Rough approximation of SNR by using avg difference + noise over noise
                uint16_t snr =
                    (high_res.avg - calibration->avg_low + calibration->noise) / calibration->noise;
                LOG_DBG("High avg for %d,%d is %d. SNR %d", s, i, high_res.avg, snr);

                calibration->avg_high = high_res.avg;
                calibration->noise = MAX(calibration->noise, high_res.noise);
                keys_to_complete--;

                if (data->calibration_callback) {
                    struct zmk_kscan_ec_matrix_calibration_event ev = {
                        .type = CALIBRATION_EV_POSITION_COMPLETE,
                        .data = {.position_complete = {.high_avg = calibration->avg_high,
                                                       .snr = snr,
                                                       .low_avg = calibration->avg_low,
                                                       .strobe = s,
                                                       .input = i,
                                                       .noise = calibration->noise}}};
                    data->calibration_callback(&ev, data->calibration_user_data);
                }

                k_sleep(K_MSEC(1));
            }

            k_sleep(K_MSEC(1));
        }

        k_sleep(K_MSEC(1));
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 0);
    }

    if (data->calibration_callback) {
        struct zmk_kscan_ec_matrix_calibration_event ev = {
            .type = CALIBRATION_EV_COMPLETE,
        };
        data->calibration_callback(&ev, data->calibration_user_data);
    }

    data->calibration_callback = NULL;
    data->calibration_user_data = NULL;
}

int zmk_kscan_ec_matrix_calibrate(const struct device *dev,
                                  zmk_kscan_ec_matrix_calibration_cb_t callback,
                                  const void *user_data) {
    struct kscan_ec_matrix_data *data = dev->data;

    int ret = k_mutex_lock(&data->mutex, K_SECONDS(1));

    if (ret < 0) {
        return -EAGAIN;
    }

    data->calibration_callback = callback;
    data->calibration_user_data = user_data;

    k_mutex_unlock(&data->mutex);

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)

int zmk_kscan_ec_matrix_access_calibration(const struct device *dev,
                                           zmk_kscan_ec_matrix_calibration_access_cb_t cb,
                                           const void *user_data) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    int ret = k_mutex_lock(&data->mutex, K_SECONDS(1));

    if (ret < 0) {
        return -EAGAIN;
    }

    cb(dev, data->calibrations, cfg->inputs_len * cfg->strobes_len, user_data);

    k_mutex_unlock(&data->mutex);

    return 0;
}

static void kscan_ec_matrix_read(const struct device *dev) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    uint64_t rows[cfg->strobes_len];

    for (int s = 0; s < cfg->strobes_len; s++) {
        rows[s] = 0;
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 1);
        k_busy_wait(cfg->matrix_warm_up_us);
    }

    for (int r = 0; r < cfg->inputs_len; r++) {
        for (int s = 0; s < cfg->strobes_len; s++) {
            struct zmk_kscan_ec_matrix_calibration_entry *calibration =
                calibration_entry_for_strobe_input(dev, s, r);

            if (!calibration || calibration->avg_high == 0) {
                continue;
            }

            if (cfg->strobe_input_masks && (cfg->strobe_input_masks[s] & BIT(r)) != 0) {
                continue;
            }

            bool prev = (data->matrix_state[s] & BIT(r)) != 0;
            uint16_t buf = read_raw_matrix_state(dev, s, r);

            buf = normalize(buf, calibration->avg_low, calibration->avg_high);

            uint16_t range = calibration->avg_high - calibration->avg_low;
            uint16_t press_limit_raw = calibration->avg_high - (range / 6);
            uint16_t hys_buffer = (range / 3);
            uint16_t press_limit =
                normalize(press_limit_raw, calibration->avg_low, calibration->avg_high);
            uint16_t release_limit = normalize(press_limit_raw - hys_buffer, calibration->avg_low,
                                               calibration->avg_high);

            if (buf > press_limit && !prev) {
                WRITE_BIT(rows[s], r, 1);
            } else if (prev && buf < release_limit) {
                WRITE_BIT(rows[s], r, 0);
            } else {
                WRITE_BIT(rows[s], r, prev);
            }

            k_yield();
        }

        k_yield();
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 0);
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    bool have_change = false;
    bool have_keys = false;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

    uint64_t diffs[cfg->strobes_len];
    for (int s = 0; s < cfg->strobes_len; s++) {
        diffs[s] = rows[s] & data->matrix_state[s];
        if (rows[s] && rows[s] != data->matrix_state[s]) {
            LOG_DBG("Initial press detected for %d/%lld", s, rows[s] ^ data->matrix_state[s]);
        }
        data->matrix_state[s] = rows[s];
    }

    for (int s = 0; s < cfg->strobes_len; s++) {
        uint64_t diff = diffs[s];
        for (int r = 0; r < cfg->inputs_len; r++) {
            if ((data->reported_matrix_state[s] & BIT(r)) != (diff & BIT(r))) {
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
                have_change = true;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

                LOG_DBG("Reporting %d/%d as %s", s, r, (diff & BIT(r)) ? "on" : "off");
                if (data->callback) {
                    data->callback(data->dev, s, r, diff & BIT(r));
                }
            } else if ((rows[s] & BIT(r)) &&
                       (data->reported_matrix_state[s] & BIT(r)) != (rows[s] & BIT(r))) {
                LOG_DBG("Bit enabled but not reporting yet %d/%d", s, r);
            }
        }

        data->reported_matrix_state[s] = diff;

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
        have_keys = have_keys || diff != 0;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    }

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    if (have_change) {
        if (have_keys) {
            data->last_key_released_at = 0;
        } else {
            data->last_key_released_at = k_uptime_get();
        }
    }
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
static void kscan_ec_matrix_update_poll_interval(const struct device *dev) {
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;

    uint32_t last_released_at = data->last_key_released_at;
    uint32_t prev_poll_interval = data->poll_interval;
    uint32_t new_poll_interval = 0;

    if (last_released_at == 0) {
        new_poll_interval = cfg->active_polling_interval_ms;
    } else {
        uint32_t ms_since_last_released = k_uptime_get() - last_released_at;

        if (ms_since_last_released > cfg->sleep_after_secs * 1000) {
            new_poll_interval = cfg->sleep_polling_interval_ms;
        } else if (ms_since_last_released > cfg->idle_after_secs * 1000) {
            new_poll_interval = cfg->idle_polling_interval_ms;
        } else {
            new_poll_interval = cfg->active_polling_interval_ms;
        }
    }

    if (new_poll_interval != prev_poll_interval) {
        LOG_WRN("Poll interval: %d -> %d", prev_poll_interval, new_poll_interval);
        data->poll_interval = new_poll_interval;
    }
}
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

uint64_t zmk_kscan_ec_matrix_max_scan_duration_ns(const struct device *dev) {
    struct kscan_ec_matrix_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_MSEC(10));

    uint64_t val = data->max_scan_duration_ns;

    k_mutex_unlock(&data->mutex);

    return val;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)

struct zmk_kscan_ec_matrix_read_timing zmk_kscan_ec_matrix_read_timing(const struct device *dev) {
    struct kscan_ec_matrix_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_MSEC(10));

    struct zmk_kscan_ec_matrix_read_timing val = data->read_timing;

    k_mutex_unlock(&data->mutex);

    return val;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)

static void kscan_ec_matrix_thread_main(void *arg1, void *unused1, void *unused2) {
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);

    const struct device *dev = (const struct device *)arg1;
    struct kscan_ec_matrix_data *data = dev->data;

    while (1) {
        k_mutex_lock(&data->mutex, K_FOREVER);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)
        if (data->calibration_callback) {
            calibrate(dev);
#else
        if (false) {
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)

        } else {
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
            timing_start();
            timing_t c1 = timing_counter_get();
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

            kscan_ec_matrix_read(dev);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
            const struct kscan_ec_matrix_config *cfg = dev->config;
            if (cfg->dynamic_polling_interval) {
                kscan_ec_matrix_update_poll_interval(dev);
            }
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
            timing_t c2 = timing_counter_get();
            uint64_t cycles = timing_cycles_get(&c1, &c2);
            uint64_t ns_spent = timing_cycles_to_ns(cycles);
            timing_stop();

            data->max_scan_duration_ns = ns_spent;
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
        }
        k_mutex_unlock(&data->mutex);
        k_sleep(K_MSEC(data->poll_interval));
    }
}

static int kscan_ec_matrix_init(const struct device *dev) {
    int err;
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;

    data->dev = dev;

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)
    data->last_key_released_at = k_uptime_get();
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
    timing_init();
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

    k_mutex_init(&data->mutex);

    if (!device_is_ready(cfg->adc_channel.dev)) {
        LOG_ERR("ADC Channel device is not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(&cfg->adc_channel);
    if (err < 0) {
        LOG_ERR("Failed to set up ADC channnel (%d)", err);
        return err;
    }

    if (!cfg->skip_startup_calibration) {
        int16_t buf = 0;
        struct adc_sequence sequence = {
            .buffer = &buf,
            .buffer_size = sizeof(buf),
        };

        adc_sequence_init_dt(&cfg->adc_channel, &sequence);
        sequence.calibrate = true;

        err = adc_read(cfg->adc_channel.dev, &sequence);
        if (err < 0) {
            LOG_ERR("Failed to calibrate on startup: %d", err);
            return err;
        }
    }

    if (cfg->pcfg) {
        err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0) {
            LOG_ERR("Failed to apply pinctrl state");
            return err;
        }
    }

    if (cfg->power.port != NULL) {
        if (!device_is_ready(cfg->power.port)) {
            LOG_ERR("Power port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT_INACTIVE);
    }

    if (cfg->drain.port != NULL) {
        if (!device_is_ready(cfg->drain.port)) {
            LOG_ERR("Drain port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->drain, GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < cfg->strobes_len; i++) {
        if (!device_is_ready(cfg->strobes[i].port)) {
            LOG_ERR("Strobe port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->strobes[i], GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < cfg->inputs_len; i++) {
        if (!device_is_ready(cfg->inputs[i].port)) {
            LOG_ERR("Input port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->inputs[i], GPIO_DISCONNECTED);
    }

    data->poll_interval = cfg->active_polling_interval_ms;

    k_mutex_lock(&data->mutex, K_MSEC(5));

    k_thread_create(&data->thread, data->thread_stack, CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_STACK_SIZE,
                    kscan_ec_matrix_thread_main, (void *)dev, NULL, NULL,
                    K_PRIO_COOP(CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_PRIORITY), 0, K_NO_WAIT);

    return 0;
}

static const struct kscan_driver_api kscan_ec_matrix_api = {
    .config = kscan_ec_matrix_configure,
    .enable_callback = kscan_ec_matrix_enable,
    .disable_callback = kscan_ec_matrix_disable,
};

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int zkem_pm_resume(const struct device *dev) { return kscan_ec_matrix_enable(dev); }

static int zkem_pm_suspend(const struct device *dev) { return kscan_ec_matrix_disable(dev); }

static int zkem_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return zkem_pm_suspend(dev);
    case PM_DEVICE_ACTION_RESUME:
        return zkem_pm_resume(dev);
    default:
        return -ENOTSUP;
    }
}

#endif //  IS_ENABLED(CONFIG_PM_DEVICE)

#define ZKEM_GPIO_DT_SPEC_ELEM(n, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),

#define ZERO(n, idx) 0

#define ENTRIES(n) DT_INST_PROP_LEN(n, strobe_gpios) * DT_INST_PROP_LEN(n, input_gpios)

#define FOREACH_STROBE_CALIB_ENTRY(n, prop, idx)                                                   \
    {.avg_low = DT_PROP_BY_IDX(n, precalib_avg_lows, idx),                                         \
     .avg_high = DT_PROP_BY_IDX(n, precalib_avg_highs, idx)}

#define ZKEM_INIT(n)                                                                               \
    PM_DEVICE_DT_INST_DEFINE(n, zkem_pm_action);                                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_names), (PINCTRL_DT_INST_DEFINE(n);), ())         \
    static struct zmk_kscan_ec_matrix_calibration_entry calibration_entries_##n[ENTRIES(n)] = {    \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, precalib_avg_lows),                                   \
                    (DT_INST_FOREACH_PROP_ELEM_SEP(n, precalib_avg_lows,                           \
                                                   FOREACH_STROBE_CALIB_ENTRY, (, ))),             \
                    (0))};                                                                         \
    static uint64_t reported_matrix_states_##n[DT_INST_PROP_LEN(n, strobe_gpios)] = {0};           \
    COND_CODE_1(                                                                                   \
        DT_INST_NODE_HAS_PROP(n, strobe_input_masks),                                              \
        (static const uint32_t strobe_input_masks_##n[] = DT_INST_PROP(n, strobe_input_masks);),   \
        ())                                                                                        \
    static struct kscan_ec_matrix_data kscan_ec_matrix_data##n = {                                 \
        .reported_matrix_state = reported_matrix_states_##n,                                       \
        .calibrations = calibration_entries_##n,                                                   \
        .matrix_state = {LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), ZERO, (, ))},                  \
    };                                                                                             \
    static const struct gpio_dt_spec inputs_##n[] = {                                              \
        DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), input_gpios, ZKEM_GPIO_DT_SPEC_ELEM)};                \
    static const struct kscan_ec_matrix_config kscan_ec_matrix_config##n = {                       \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_names),                                       \
                    (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), ), ())                             \
            .adc_channel = ADC_DT_SPEC_INST_GET(n),                                                \
        .power = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                                    \
        .drain = GPIO_DT_SPEC_INST_GET_OR(n, drain_gpios, {0}),                                    \
        .strobes = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), strobe_gpios, ZKEM_GPIO_DT_SPEC_ELEM)},   \
        .strobes_len = DT_INST_PROP_LEN(n, strobe_gpios),                                          \
        .inputs = inputs_##n,                                                                      \
        .inputs_len = DT_INST_PROP_LEN(n, input_gpios),                                            \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, strobe_input_masks),                                  \
                    (.strobe_input_masks = strobe_input_masks_##n, ), ())                          \
            .matrix_warm_up_us = DT_INST_PROP_OR(n, matrix_warm_up_us, 0),                         \
        .matrix_relax_us = DT_INST_PROP_OR(n, matrix_relax_us, 0),                                 \
        .adc_read_settle_us = DT_INST_PROP_OR(n, adc_read_settle_us, 0),                           \
        .active_polling_interval_ms = DT_INST_PROP_OR(n, active_polling_interval_ms, 1),           \
        .skip_startup_calibration = DT_INST_PROP_OR(n, skip_startup_calibration, false),           \
        COND_CODE_1(                                                                               \
            IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_DYNAMIC_POLL_RATE),                              \
            (.idle_polling_interval_ms = DT_INST_PROP_OR(n, idle_polling_interval_ms, 5),          \
             .sleep_polling_interval_ms = DT_INST_PROP_OR(n, sleep_polling_interval_ms, 500),      \
             .idle_after_secs = DT_INST_PROP_OR(n, idle_after_secs, 5),                            \
             .sleep_after_secs = DT_INST_PROP_OR(n, sleep_after_secs, 300),                        \
             .dynamic_polling_interval = DT_INST_PROP_OR(n, dynamic_polling_interval, false), ),   \
            ())};                                                                                  \
    DEVICE_DT_INST_DEFINE(n, kscan_ec_matrix_init, PM_DEVICE_DT_INST_GET(n),                       \
                          &kscan_ec_matrix_data##n, &kscan_ec_matrix_config##n, POST_KERNEL,       \
                          CONFIG_KSCAN_INIT_PRIORITY, &kscan_ec_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(ZKEM_INIT)
