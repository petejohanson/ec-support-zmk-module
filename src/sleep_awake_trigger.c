
#include <ec_support/drivers/misc/adxl362.h>

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include <zephyr/sys/reboot.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_DECLARE(ADXL362_AWAKE_TRIGGER, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *dev = DEVICE_DT_GET(DT_INST(0, zmk_adxl362_awake_trigger));

static int sleep_awake_trigger_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    int ret;

    if (!ev) {
        return -ENOTSUP;
    }

    if (ev->state != ZMK_ACTIVITY_SLEEP) {
        return 0;
    }

    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        adxl362_awake_trigger_set_activity_limit(dev, ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_NORMAL);
        break;
    case ZMK_ACTIVITY_SLEEP:
        ret = adxl362_awake_trigger_set_activity_limit(dev,
                                                       ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_SLEEP);
        if (ret < 0) {
            switch (ret) {
            case -ECANCELED:
                sys_reboot(SYS_REBOOT_WARM);
                break;
            default:
                break;
            }

        } else {
            // LOG_PANIC();
            // k_sleep(K_SECONDS(10));
        }

        break;
    default:
        break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(sleep_awake_trigger, sleep_awake_trigger_listener);
ZMK_SUBSCRIPTION(sleep_awake_trigger, zmk_activity_state_changed);
