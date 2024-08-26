/* ADXL362 Awake Trigger */

/*
 * Copyright (c) 2024 Pete Johanson
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_adxl362_awake_trigger

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <ec_support/drivers/misc/adxl362.h>

#include "adxl362.h"

#if IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)

static const struct device *led_dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(led0)));
static uint8_t led_idx = DT_NODE_CHILD_IDX(DT_ALIAS(led0));

#endif // IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)

LOG_MODULE_REGISTER(ADXL362_AWAKE_TRIGGER, CONFIG_LOG_DEFAULT_LEVEL);

static int zaat_reg_access(const struct device *dev, uint8_t cmd, uint8_t reg_addr, void *data,
                           size_t length) {
    const struct zaat_config *cfg = dev->config;
    uint8_t access[2] = {cmd, reg_addr};
    const struct spi_buf buf[2] = {{.buf = access, .len = 2}, {.buf = data, .len = length}};
    struct spi_buf_set tx = {
        .buffers = buf,
    };

    if (cmd == ADXL362_READ_REG) {
        const struct spi_buf_set rx = {.buffers = buf, .count = 2};

        tx.count = 1;

        return spi_transceive_dt(&cfg->bus, &tx, &rx);
    }

    tx.count = 2;

    return spi_write_dt(&cfg->bus, &tx);
}

static inline int zaat_set_reg(const struct device *dev, uint16_t register_value,
                               uint8_t register_address, uint8_t count) {
    return zaat_reg_access(dev, ADXL362_WRITE_REG, register_address, &register_value, count);
}

int zaat_reg_write_mask(const struct device *dev, uint8_t register_address, uint8_t mask,
                        uint8_t data) {
    int ret;
    uint8_t tmp;

    ret = zaat_reg_access(dev, ADXL362_READ_REG, register_address, &tmp, 1);

    if (ret) {
        return ret;
    }

    tmp &= ~mask;
    tmp |= data;

    return zaat_reg_access(dev, ADXL362_WRITE_REG, register_address, &tmp, 1);
}

static inline int zaat_get_reg(const struct device *dev, uint8_t *read_buf,
                               uint8_t register_address, uint8_t count) {

    return zaat_reg_access(dev, ADXL362_READ_REG, register_address, read_buf, count);
}

static int zaat_interrupt_config(const struct device *dev, uint8_t int1, uint8_t int2) {
    int ret;

    ret = zaat_reg_access(dev, ADXL362_WRITE_REG, ADXL362_REG_INTMAP1, &int1, 1);

    if (ret) {
        return ret;
    }

    return ret = zaat_reg_access(dev, ADXL362_WRITE_REG, ADXL362_REG_INTMAP2, &int2, 1);
}

// static int zaat_get_status(const struct device *dev, uint8_t *status) {
//     return zaat_get_reg(dev, status, ADXL362_REG_STATUS, 1);
// }

static int zaat_software_reset(const struct device *dev) {
    return zaat_set_reg(dev, ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET, 1);
}

static int zaat_set_range(const struct device *dev, uint8_t range) {
    struct zaat_data *zaat_data = dev->data;
    uint8_t old_filter_ctl;
    uint8_t new_filter_ctl;
    int ret;

    ret = zaat_get_reg(dev, &old_filter_ctl, ADXL362_REG_FILTER_CTL, 1);
    if (ret) {
        return ret;
    }

    new_filter_ctl = old_filter_ctl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    new_filter_ctl = new_filter_ctl | ADXL362_FILTER_CTL_RANGE(range);
    ret = zaat_set_reg(dev, new_filter_ctl, ADXL362_REG_FILTER_CTL, 1);
    if (ret) {
        return ret;
    }

    zaat_data->selected_range = range;
    return 0;
}

static int zaat_set_output_rate(const struct device *dev, uint8_t out_rate) {
    int ret;
    uint8_t old_filter_ctl;
    uint8_t new_filter_ctl;

    ret = zaat_get_reg(dev, &old_filter_ctl, ADXL362_REG_FILTER_CTL, 1);
    if (ret) {
        return ret;
    }

    new_filter_ctl = old_filter_ctl & ~ADXL362_FILTER_CTL_ODR(0x7);
    new_filter_ctl = new_filter_ctl | ADXL362_FILTER_CTL_ODR(out_rate);
    return zaat_set_reg(dev, new_filter_ctl, ADXL362_REG_FILTER_CTL, 1);
}

static int zaat_fifo_setup(const struct device *dev, uint8_t mode, uint16_t water_mark_lvl,
                           uint8_t en_temp_read) {
    uint8_t write_val;
    int ret;

    write_val = ADXL362_FIFO_CTL_FIFO_MODE(mode) | (en_temp_read * ADXL362_FIFO_CTL_FIFO_TEMP) |
                ADXL362_FIFO_CTL_AH;
    ret = zaat_set_reg(dev, write_val, ADXL362_REG_FIFO_CTL, 1);
    if (ret) {
        return ret;
    }

    ret = zaat_set_reg(dev, water_mark_lvl, ADXL362_REG_FIFO_SAMPLES, 1);
    if (ret) {
        return ret;
    }

    return 0;
}

static int zaat_setup_activity_detection(const struct device *dev, uint8_t ref_or_abs,
                                         uint16_t threshold, uint8_t time) {
    uint8_t old_act_inact_reg;
    uint8_t new_act_inact_reg;
    int ret;

    /**
     * mode
     *              must be one of the following:
     *			ADXL362_FIFO_DISABLE      -  FIFO is disabled.
     *			ADXL362_FIFO_OLDEST_SAVED -  Oldest saved mode.
     *			ADXL362_FIFO_STREAM       -  Stream mode.
     *			ADXL362_FIFO_TRIGGERED    -  Triggered mode.
     * water_mark_lvl
     *              Specifies the number of samples to store in the FIFO.
     * en_temp_read
     *              Store Temperature Data to FIFO.
     *              1 - temperature data is stored in the FIFO
     *                  together with x-, y- and x-axis data.
     *          0 - temperature data is skipped.
     */

    /* Configure motion threshold and activity timer. */
    ret = zaat_set_reg(dev, (threshold & 0x7FF), ADXL362_REG_THRESH_ACT_L, 2);
    if (ret) {
        return ret;
    }

    ret = zaat_set_reg(dev, time, ADXL362_REG_TIME_ACT, 1);
    if (ret) {
        return ret;
    }

    /* Enable activity interrupt and select a referenced or absolute
     * configuration.
     */
    ret = zaat_get_reg(dev, &old_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);
    if (ret) {
        return ret;
    }

    new_act_inact_reg = old_act_inact_reg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    new_act_inact_reg |=
        ADXL362_ACT_INACT_CTL_ACT_EN | (ref_or_abs * ADXL362_ACT_INACT_CTL_ACT_REF);
    LOG_WRN("New ACT/INACT reg 0x%02x", new_act_inact_reg);
    ret = zaat_set_reg(dev, new_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);
    if (ret) {
        return ret;
    }

    return 0;
}

static int zaat_setup_inactivity_detection(const struct device *dev, uint8_t ref_or_abs,
                                           uint16_t threshold, uint16_t time) {
    uint8_t old_act_inact_reg;
    uint8_t new_act_inact_reg;
    int ret;

    /* Configure motion threshold and inactivity timer. */
    ret = zaat_set_reg(dev, (threshold & 0x7FF), ADXL362_REG_THRESH_INACT_L, 2);
    LOG_ERR("Set the threshold: %d", ret);
    if (ret) {
        return ret;
    }

    ret = zaat_set_reg(dev, time, ADXL362_REG_TIME_INACT_L, 2);
    LOG_ERR("Set the inactivity time: %d", ret);
    if (ret) {
        return ret;
    }

    /* Enable inactivity interrupt and select a referenced or
     * absolute configuration.
     */
    ret = zaat_get_reg(dev, &old_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);
    if (ret) {
        return ret;
    }

    new_act_inact_reg = old_act_inact_reg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    new_act_inact_reg |=
        ADXL362_ACT_INACT_CTL_INACT_EN | (ref_or_abs * ADXL362_ACT_INACT_CTL_INACT_REF);
    ret = zaat_set_reg(dev, new_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);
    if (ret) {
        return ret;
    }

    return 0;
}

int zaat_set_interrupt_mode(const struct device *dev, uint8_t mode) {
    uint8_t old_act_inact_reg;
    uint8_t new_act_inact_reg;
    int ret;

    LOG_DBG("Mode: %d", mode);

    if (mode != ADXL362_MODE_DEFAULT && mode != ADXL362_MODE_LINK && mode != ADXL362_MODE_LOOP) {
        LOG_ERR("Wrong mode");
        return -EINVAL;
    }

    /* Select desired interrupt mode. */
    ret = zaat_get_reg(dev, &old_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);
    if (ret) {
        return ret;
    }

    new_act_inact_reg = old_act_inact_reg & ~ADXL362_ACT_INACT_CTL_LINKLOOP(3);
    new_act_inact_reg |= old_act_inact_reg | ADXL362_ACT_INACT_CTL_LINKLOOP(mode);

    ret = zaat_set_reg(dev, new_act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);

    if (ret) {
        return ret;
    }

    return 0;
}

static inline int zaat_range_to_scale(int range) {
    /* See table 1 in specifications section of datasheet */
    switch (range) {
    case ADXL362_RANGE_2G:
        return ADXL362_ACCEL_2G_LSB_PER_G;
    case ADXL362_RANGE_4G:
        return ADXL362_ACCEL_4G_LSB_PER_G;
    case ADXL362_RANGE_8G:
        return ADXL362_ACCEL_8G_LSB_PER_G;
    default:
        return -EINVAL;
    }
}

static int zaat_set_awake_config(const struct device *dev,
                                 const struct zaat_awake_config *awake_config) {
    int ret;

    /* Configures activity detection.
     *	Referenced/Absolute Activity or Inactivity Select.
     *		0 - absolute mode.
     *		1 - referenced mode.
     *	threshold
     *		11-bit unsigned value that the adxl362 samples are
     *		compared to.
     *	time
     *		8-bit value written to the activity timer register.
     *		The amount of time (in seconds) is:
     *			time / ODR,
     *		where ODR - is the output data rate.
     */
    ret = zaat_setup_activity_detection(dev, 1, awake_config->activity_threshold,
                                        awake_config->activity_time);
    if (ret) {
        return ret;
    }

    /* Configures inactivity detection.
     *	Referenced/Absolute Activity or Inactivity Select.
     *		0 - absolute mode.
     *		1 - referenced mode.
     *	threshold
     *		11-bit unsigned value that the adxl362 samples are
     *		compared to.
     *	time
     *		16-bit value written to the activity timer register.
     *		The amount of time (in seconds) is:
     *			time / ODR,
     *		where ODR - is the output data rate.
     */
    return zaat_setup_inactivity_detection(dev, 1, awake_config->inactivity_threshold,
                                           awake_config->inactivity_time);
}

int adxl362_awake_trigger_set_activity_limit(const struct device *dev,
                                             enum adxl362_awake_trigger_activity_limit limit) {
    const struct zaat_config *cfg = dev->config;
    struct zaat_data *data = dev->data;

    int ret = -ENOTSUP;
    switch (limit) {
    case ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_NORMAL:
        ret = zaat_set_awake_config(dev, &cfg->normal_awake_config);
        if (ret < 0) {
            LOG_WRN("Failed to set normal awake config");
        }
        ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_BOTH);
    case ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_SLEEP:
        gpio_remove_callback(cfg->interrupt.port, &data->gpio_cb);
        ret = zaat_set_awake_config(dev, &cfg->sleep_awake_config);

        // After setting the config, disable then resume measurement/autosleep, etc. to ensure
        // new sleep config is applied and in effect
        ret = zaat_set_reg(dev, ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_STANDBY),
                           ADXL362_REG_POWER_CTL, 1);
        ret = zaat_set_reg(dev,
                           ADXL362_POWER_CTL_AUTOSLEEP | ADXL362_POWER_CTL_WAKEUP |
                               ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON),
                           ADXL362_REG_POWER_CTL, 1);

        if (ret >= 0) {
            k_sleep(K_MSEC(50));
            int tries_left = 5;
            while ((tries_left--) >= 0 && gpio_pin_get_dt(&cfg->interrupt) > 0) {
                uint8_t status, ctl;
                zaat_get_reg(dev, &status, ADXL362_REG_STATUS, 1);
                zaat_get_reg(dev, &ctl, ADXL362_REG_ACT_INACT_CTL, 1);
                k_sleep(K_MSEC(50));
            }

            if (tries_left < 0) {
                LOG_ERR("Failed to set into inactive state with current settings");
                return -ECANCELED;
            }

            ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_LEVEL_ACTIVE);
        }
    }

    return ret;
}

static int zaat_chip_init(const struct device *dev) {
    const struct zaat_config *config = dev->config;
    int ret;

    zaat_set_awake_config(dev, &config->normal_awake_config);

    /* Configures the FIFO feature. */
    ret = zaat_fifo_setup(dev, ADXL362_FIFO_DISABLE, 0, 0);
    if (ret) {
        return ret;
    }

    /* Selects the measurement range.
     * options are:
     *		ADXL362_RANGE_2G  -  +-2 g
     *		ADXL362_RANGE_4G  -  +-4 g
     *		ADXL362_RANGE_8G  -  +-8 g
     */
    ret = zaat_set_range(dev, ADXL362_RANGE_2G);
    if (ret) {
        return ret;
    }

    /* Selects the Output Data Rate of the device.
     * Options are:
     *		ADXL362_ODR_12_5_HZ  -  12.5Hz
     *		ADXL362_ODR_25_HZ    -  25Hz
     *		ADXL362_ODR_50_HZ    -  50Hz
     *		ADXL362_ODR_100_HZ   -  100Hz
     *		ADXL362_ODR_200_HZ   -  200Hz
     *		ADXL362_ODR_400_HZ   -  400Hz
     */
    ret = zaat_set_output_rate(dev, ADXL362_ODR_12_5_HZ);
    if (ret) {
        return ret;
    }

    /* Places the device into measure mode, enable wakeup mode and autosleep if desired. */
    // LOG_WRN("setting pwrctl: 0x%02x", ADXL362_POWER_CTL_WAKEUP |
    // ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));
    ret = zaat_set_reg(dev,
                       ADXL362_POWER_CTL_WAKEUP | ADXL362_POWER_CTL_AUTOSLEEP |
                           ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON),
                       ADXL362_REG_POWER_CTL, 1);
    LOG_WRN("Set the power ctl: %d", ret);
    if (ret) {
        return ret;
    }

    return 0;
}

static void suspend_devices_cb(struct k_work *work) {
    struct zaat_data *drv_data = CONTAINER_OF(work, struct zaat_data, suspend_work);

    const struct device *my_dev = drv_data->dev;
    const struct zaat_config *cfg = my_dev->config;

    for (int i = 0; i < cfg->linked_devices_size; i++) {
        pm_device_action_run(cfg->linked_devices[i], PM_DEVICE_ACTION_SUSPEND);
    }
}

static void resume_devices_cb(struct k_work *work) {
    struct zaat_data *drv_data = CONTAINER_OF(work, struct zaat_data, resume_work);

    const struct device *my_dev = drv_data->dev;
    const struct zaat_config *cfg = my_dev->config;

    for (int i = 0; i < cfg->linked_devices_size; i++) {
        pm_device_action_run(cfg->linked_devices[i], PM_DEVICE_ACTION_RESUME);
    }
}

static void zaat_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct zaat_data *drv_data = CONTAINER_OF(cb, struct zaat_data, gpio_cb);

    const struct device *my_dev = drv_data->dev;

    const struct zaat_config *cfg = my_dev->config;

    int val = gpio_pin_get_dt(&cfg->interrupt);

    if (val) {
#if IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)
        led_off(led_dev, led_idx);
#endif // IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)
        k_work_cancel(&drv_data->suspend_work);
        k_work_submit(&drv_data->resume_work);
    } else {
#if IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)
        led_on(led_dev, led_idx);
#endif // IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_LED_DEBUG)
        k_work_cancel(&drv_data->resume_work);
        k_work_submit(&drv_data->suspend_work);
    }
}

static int zaat_init_interrupt(const struct device *dev) {
    const struct zaat_config *cfg = dev->config;
    struct zaat_data *drv_data = dev->data;
    int ret;

    if (!gpio_is_ready_dt(&cfg->interrupt)) {
        LOG_ERR("GPIO port %s not ready", cfg->interrupt.port->name);
        return -ENODEV;
    }

    ret = zaat_set_interrupt_mode(dev, ADXL362_MODE_LOOP);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }

    gpio_init_callback(&drv_data->gpio_cb, zaat_gpio_callback, BIT(cfg->interrupt.pin));

    ret = gpio_add_callback(cfg->interrupt.port, &drv_data->gpio_cb);
    if (ret < 0) {
        return ret;
    }

    drv_data->dev = dev;

    ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Initializes communication with the device and checks if the part is
 *        present by reading the device id.
 *
 * @return  0 - the initialization was successful and the device is present;
 *         -1 - an error occurred.
 *
 */
static int zaat_init(const struct device *dev) {
    const struct zaat_config *config = dev->config;
    struct zaat_data *data = dev->data;
    uint8_t value = 0;
    int err;

    if (!spi_is_ready_dt(&config->bus)) {
        LOG_DBG("spi device not ready: %s", config->bus.bus->name);
        return -EINVAL;
    }

    k_work_init(&data->suspend_work, suspend_devices_cb);
    k_work_init(&data->resume_work, resume_devices_cb);

    err = zaat_software_reset(dev);

    if (err) {
        LOG_ERR("zaat_software_reset failed, error %d", err);
        return -ENODEV;
    }

    k_sleep(K_MSEC(1));

    (void)zaat_get_reg(dev, &value, ADXL362_REG_PARTID, 1);
    if (value != ADXL362_PART_ID) {
        LOG_ERR("wrong part_id: %d", value);
        return -ENODEV;
    }

    if (zaat_chip_init(dev) < 0) {
        return -ENODEV;
    }

    if (config->interrupt.port) {
        if (zaat_init_interrupt(dev) < 0) {
            LOG_ERR("Failed to initialize interrupt!");
            return -EIO;
        }

        if (zaat_interrupt_config(dev, ADXL362_INTMAP1_AWAKE, ADXL362_INTMAP2_AWAKE) < 0) {
            LOG_ERR("Failed to configure interrupt");
            return -EIO;
        }
    }

    uint8_t status;
    err = zaat_get_reg(dev, &status, ADXL362_REG_STATUS, 1);

    if (err >= 0) {
        LOG_WRN("After setup, status is 0x%02x", status);
    }

    k_sleep(K_MSEC(10));
    err = zaat_get_reg(dev, &status, ADXL362_REG_STATUS, 1);

    if (err >= 0) {
        LOG_WRN("After setup, status is 0x%02x", status);
    }

    uint8_t act_inact_reg;
    err = zaat_get_reg(dev, &act_inact_reg, ADXL362_REG_ACT_INACT_CTL, 1);

    if (err >= 0) {
        LOG_WRN("After setup, act/inact is 0x%02x", act_inact_reg);
    }

    uint8_t xyz[3];
    err = zaat_get_reg(dev, xyz, ADXL362_REG_XDATA_H, 3);
    LOG_HEXDUMP_WRN(xyz, 3, "Data");

    return 0;
}

#define GET_LINKED_DEV(idx, inst) DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, linked_devices, idx))

#define ZAAT_DEFINE(inst)                                                                          \
    static struct zaat_data zaat_data_##inst;                                                      \
    static const struct device *linked_devices_##inst[] = {                                        \
        LISTIFY(DT_INST_PROP_LEN(inst, linked_devices), GET_LINKED_DEV, (, ), inst)};              \
                                                                                                   \
    static const struct zaat_config zaat_config_##inst = {                                         \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),                  \
        .normal_awake_config = ZAAT_AWAKE_CONFIG(inst, normal),                                    \
        COND_CODE_1(IS_ENABLED(CONFIG_ZMK_ADXL362_AWAKE_TRIGGER_SLEEP),                            \
                    (.sleep_awake_config = ZAAT_AWAKE_CONFIG(inst, sleep), ), ())                  \
            .interrupt = COND_CODE_1(DT_INST_HAS_PROP(inst, int1_gpios),                           \
                                     (GPIO_DT_SPEC_INST_GET(inst, int1_gpios)),                    \
                                     (GPIO_DT_SPEC_INST_GET(inst, int2_gpios))),                   \
        .linked_devices = linked_devices_##inst,                                                   \
        .linked_devices_size = DT_INST_PROP_LEN(inst, linked_devices),                             \
    };                                                                                             \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, zaat_init, NULL, &zaat_data_##inst, &zaat_config_##inst,    \
                                 POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ZAAT_DEFINE)
