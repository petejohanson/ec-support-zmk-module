

#define DT_DRV_COMPAT zmk_gpio_mux

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_gpio_mux);

struct zgm_config {
    /* gpio_driver_data needs to be first */
    struct gpio_driver_config common;

    struct gpio_dt_spec en_gpio;
    struct gpio_dt_spec out_gpio;
    uint8_t sel_gpios_len;
    struct gpio_dt_spec sel_gpios[];
};

struct zgm_data {
    struct gpio_driver_data common;
    gpio_pin_t active_pin;
};

static int zgm_pin_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags) {
    int ret = 0;
    const struct zgm_config *cfg = dev->config;
    struct zgm_data *data = dev->data;

    if (flags & GPIO_OUTPUT) {
        return -ENOTSUP;
    } else if (flags & GPIO_INPUT) {
        data->active_pin = pin;
        for (int i = 0; i < cfg->sel_gpios_len; i++) {
            int val = (pin & BIT(i)) != 0 ? 1 : 0;
            gpio_pin_set_dt(&cfg->sel_gpios[i], val);
        }

        if (cfg->en_gpio.port) {
            ret = gpio_pin_set_dt(&cfg->en_gpio, 1);
        }
    } else {
        if (cfg->en_gpio.port) {
            ret = gpio_pin_set_dt(&cfg->en_gpio, 0);
            if (ret < 0) {
                LOG_ERR("Failed to disable the en-gpio");
                return ret;
            }
        }
        data->active_pin = -1;
    }

    return ret;
}



static const struct gpio_driver_api api_table = {
    .pin_configure = zgm_pin_config,
    // .port_get_raw = zgm_port_get_raw,
    // .port_set_masked_raw = zgm_port_set_masked_raw,
    // .port_set_bits_raw = zgm_port_set_bits_raw,
    // .port_clear_bits_raw = zgm_port_clear_bits_raw,
    // .port_toggle_bits = zgm_port_toggle_bits,
};

/**
 * @brief Initialization function of 595
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int zgm_init(const struct device *dev) {
    const struct zgm_config *cfg = dev->config;

    if (cfg->en_gpio.port != NULL) {
        if (!device_is_ready(cfg->en_gpio.port)) {
            LOG_ERR("Enable port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->en_gpio, GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < cfg->sel_gpios_len; i++) {
        if (!device_is_ready(cfg->sel_gpios[i].port)) {
            LOG_ERR("Select port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->sel_gpios[i], GPIO_OUTPUT_INACTIVE);
    }

    return 0;
}

#define GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios) ((gpio_port_pins_t)(((uint64_t)1 << (ngpios)) - 1U))

#define GPIO_PORT_PIN_MASK_FROM_DT_INST(inst)                                                      \
    GPIO_PORT_PIN_MASK_FROM_NGPIOS((1 << DT_INST_PROP_LEN(inst, select_gpios)))

#define ZGM_GPIO_DT_SPEC_ELEM(n, prop, idx) \
    GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),

#define ZGM_INIT(n)                                                                            \
    static struct zgm_config zgm_##n##_config = {                                          \
        .common =                                                                                  \
            {                                                                                      \
                .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),                               \
            },                                                                                     \
        .en_gpio = GPIO_DT_SPEC_INST_GET_OR(n, en_gpios, {}), \
        .out_gpio = GPIO_DT_SPEC_INST_GET_OR(n, out_gpios, {0}), \
        .sel_gpios = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), select_gpios, ZGM_GPIO_DT_SPEC_ELEM)}, \
        .sel_gpios_len = DT_INST_PROP_LEN(n, select_gpios), \
    };                                                                                             \
                                                                                                   \
    static struct zgm_data zgm_##n##_data = {};                                     \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, zgm_init, NULL, &zgm_##n##_data, &zgm_##n##_config,    \
                          POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &api_table);


DT_INST_FOREACH_STATUS_OKAY(ZGM_INIT)
