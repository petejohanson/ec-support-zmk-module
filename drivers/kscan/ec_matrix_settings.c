
#include <sys/types.h>
#include <zephyr/settings/settings.h>

#include "ec_matrix_settings.h"
#include "zmk_kscan_ec_matrix.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_kscan_ec_matrix_settings);

#define MAX_SETTING_LEN 32

struct load_state {
    char setting_name[MAX_SETTING_LEN];
    struct zmk_kscan_ec_matrix_calibration_entry *entries;
    size_t len;
};

static int settings_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                            void *param) {
    struct load_state *state = (struct load_state *)param;

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_DISCRETE)

    if (len != sizeof(struct zmk_kscan_ec_matrix_calibration_entry)) {
        LOG_WRN("Ignoring settings with incorrect size");
        return -EINVAL;
    }

    char *endptr;
    size_t entry_id = strtoul(key, &endptr, 10);
    if (endptr != key) {
        if (entry_id >= state->len) {
            LOG_WRN("Ignoring calibration for invalid index %d", entry_id);
            return -EINVAL;
        }
        ssize_t ret = read_cb(cb_arg, &state->entries[entry_id], len);
        if (ret < 0) {
            LOG_ERR("Failed to load the settings from flash");
            return ret;
        }
    }
#else
    ssize_t ret = read_cb(cb_arg, state->entries,
                          state->len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
    if (ret < 0) {
        LOG_ERR("Failed to load the settings from flash");
    }

    return ret;
#endif
    return 0;
}

static void load_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries,
                    size_t len, const void *user_data) {
    struct load_state state = (struct load_state){.entries = entries, .len = len};
    snprintf(state.setting_name, MAX_SETTING_LEN, "zmk/ec/calibration/%s", dev->name);
    LOG_DBG("Loading the subtree directly for %s", state.setting_name);
    settings_load_subtree_direct(state.setting_name, settings_load_cb, &state);
}

static void save_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries,
                    size_t len, const void *user_data) {
    char setting_name[MAX_SETTING_LEN];

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_DISCRETE)
    for (size_t i = 0; i < len; i++) {
        snprintf(setting_name, MAX_SETTING_LEN, "zmk/ec/calibration/%s/%d", dev->name, i);
        int ret = settings_save_one(setting_name, &entries[i],
                                    sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
        if (ret != 0) {
            LOG_WRN("Failed to save the settings for %s: %d", setting_name, ret);
            break;
        }
    }
#else
    snprintf(setting_name, MAX_SETTING_LEN, "zmk/ec/calibration/%s", dev->name);

    int ret = settings_save_one(setting_name, entries,
                                len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));

    if (ret != 0) {
        LOG_WRN("Failed to save the settings for %s: %d", setting_name, ret);
    }
#endif
}

int zmk_kscan_ec_matrix_settings_load_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &load_cb, NULL);
    return ret;
}

int zmk_kscan_ec_matrix_settings_save_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &save_cb, NULL);
    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_INIT_LOAD)
#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define LOAD_CALL(n) zmk_kscan_ec_matrix_settings_load_calibration(DEVICE_DT_GET(DT_DRV_INST(n)));

static int zmk_kscan_ec_matrix_settings_init(void) {
    // TODO: Avoid duplicating this from the ZMK call to init?
    settings_subsys_init();
    DT_INST_FOREACH_STATUS_OKAY(LOAD_CALL)

    return 0;
}

SYS_INIT(zmk_kscan_ec_matrix_settings_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_INIT_LOAD)
