/*
 * Copyright (c) 2018 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "ec_matrix_settings.h"
#include "zmk_kscan_ec_matrix.h"

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ec_matrix_shell);

#define CMD_HELP_SCAN_RATE "Print EC Scan Rate.\n"
#define CMD_HELP_READ_TIMING "Print EC Read Timing.\n"
#define CMD_HELP_CALIBRATE "EC Calibration Utilities.\n"
#define CMD_HELP_CALIBRATION_START "Calibrate the EC Martix.\n"
#define CMD_HELP_CALIBRATION_EXPORT "Export calibration data as DTS props.\n"

#define CMD_HELP_CALIBRATION_SAVE "Save the EC Martix Calibration To Flash.\n"

#define CMD_HELP_CALIBRATION_LOAD "Load the EC Martix Calibration From Flash.\n"

#define DEVICES(n) DEVICE_DT_INST_GET(n),

#define INIT_MACRO() DT_INST_FOREACH_STATUS_OKAY(DEVICES) NULL

#define EC_MATRIX_ENTRY(dev_) {.dev = dev_}

/* This table size is = ADC devices count + 1 (NA). */
static struct matrix_hdl {
    const struct device *dev;
} matrix_hdl_list[] = {FOR_EACH(EC_MATRIX_ENTRY, (, ), INIT_MACRO())};

static struct matrix_hdl *get_matrix(const char *device_label) {
    for (int i = 0; i < ARRAY_SIZE(matrix_hdl_list); i++) {
        if (!strcmp(device_label, matrix_hdl_list[i].dev->name)) {
            return &matrix_hdl_list[i];
        }
    }

    /* This will never happen because ADC was prompted by shell */
    __ASSERT_NO_MSG(false);
    return NULL;
}

static void calibrate_cb(const struct zmk_kscan_ec_matrix_calibration_event *ev,
                         const void *user_data) {
    const struct shell *sh = (const struct shell *)user_data;

    switch (ev->type) {
    case CALIBRATION_EV_LOW_SAMPLING_START:
        shell_prompt_change(sh, "-");
        shell_print(sh, "Low value sampling begins. Please do not press any keys");
        k_sleep(K_SECONDS(1));
        break;
    case CALIBRATION_EV_HIGH_SAMPLING_START:
        shell_prompt_change(sh, "-");
        shell_print(sh, "\nHigh value sampling begins. Please slowly press each key in sequence, "
                        "releasing once an asterisk appears");
        break;
    case CALIBRATION_EV_POSITION_LOW_DETERMINED:
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_VERBOSE_CALIBRATOR)
        shell_print(sh, "Key at (%d,%d) is calibrated with avg low %d, noise: %d",
                    ev->data.position_low_determined.strobe, ev->data.position_low_determined.input,
                    ev->data.position_low_determined.low_avg,
                    ev->data.position_low_determined.noise);
#else
        shell_fprintf(sh, SHELL_NORMAL, "*");
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_VERBOSE_CALIBRATOR)
        break;
    case CALIBRATION_EV_POSITION_COMPLETE:
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_VERBOSE_CALIBRATOR)
        shell_print(sh,
                    "Key at (%d,%d) is calibrated with avg low %d, avg high %d, noise: %d, SNR: %d",
                    ev->data.position_complete.strobe, ev->data.position_complete.input,
                    ev->data.position_complete.low_avg, ev->data.position_complete.high_avg,
                    ev->data.position_complete.noise, ev->data.position_complete.snr);
#else
        shell_fprintf(sh, SHELL_NORMAL, "*");
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_VERBOSE_CALIBRATOR)
        break;
    case CALIBRATION_EV_COMPLETE:
        shell_prompt_change(sh, CONFIG_SHELL_PROMPT_UART);
        shell_print(sh, "\nCalibration complete!");
        break;
    }
}

static int cmd_matrix_calibration_start(const struct shell *shell, size_t argc, char **argv,
                                        void *data) {
    /* -2: index of ADC label name */
    struct matrix_hdl *matrix = get_matrix(argv[-2]);

    int ret = zmk_kscan_ec_matrix_calibrate(matrix->dev, &calibrate_cb, shell);
    if (ret < 0) {
        shell_print(shell, "Failed to start calibration (%d)", ret);
    }

    return ret;
}

static void export_cb(const struct device *dev,
                      struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len,
                      const void *user_data) {
    const struct shell *shell = (const struct shell *)user_data;

    shell_print(shell, "\tprecalib-avg-highs = <");
    for (size_t i = 0; i < len; i++) {
        shell_print(shell, "\t\t%d", entries[i].avg_high);
    }
    shell_print(shell, "\t>;");
    shell_print(shell, "precalib-avg-lows = <");
    for (size_t i = 0; i < len; i++) {
        shell_print(shell, "\t\t%d", entries[i].avg_low);
    }
    shell_print(shell, "\t>;");
}

static int cmd_matrix_calibration_export(const struct shell *shell, size_t argc, char **argv,
                                         void *data) {
    /* -2: index of ADC label name */
    struct matrix_hdl *matrix = get_matrix(argv[-2]);

    int ret = zmk_kscan_ec_matrix_access_calibration(matrix->dev, &export_cb, shell);
    if (ret < 0) {
        shell_print(shell, "Failed to access calibration data to export (%d)", ret);
    }

    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

static int cmd_matrix_scan_rate(const struct shell *shell, size_t argc, char **argv, void *data) {
    struct matrix_hdl *matrix = get_matrix(argv[-1]);
    uint64_t duration_ns = zmk_kscan_ec_matrix_max_scan_duration_ns(matrix->dev);

    if (duration_ns > 0) {
        uint64_t scan_rate = 1000000000 / duration_ns;
        shell_info(shell, "Matrix scan rate: %lluHz", scan_rate);
    }

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)

static void print_pct(const struct shell *shell, uint64_t total_ns, uint64_t subset_ns,
                      const char *label) {
    uint32_t pct = (subset_ns * 100) / total_ns;

    shell_print(shell, "%s: %u%%", label, pct);
}

static int cmd_matrix_read_timing(const struct shell *shell, size_t argc, char **argv, void *data) {
    struct matrix_hdl *matrix = get_matrix(argv[-1]);
    struct zmk_kscan_ec_matrix_read_timing timing = zmk_kscan_ec_matrix_read_timing(matrix->dev);

    shell_print(shell, "Total time for a read: %lluns", timing.total_ns);
    print_pct(shell, timing.total_ns, timing.adc_sequence_init_ns, "Sequence Init");
    print_pct(shell, timing.total_ns, timing.gpio_input_ns, "GPIO Input");
    print_pct(shell, timing.total_ns, timing.relax_ns, "Relax");
    print_pct(shell, timing.total_ns, timing.plug_drain_ns, "Plug Drain");
    print_pct(shell, timing.total_ns, timing.set_strobe_ns, "Set Strobe");
    print_pct(shell, timing.total_ns, timing.read_settle_ns, "Read Settle");
    print_pct(shell, timing.total_ns, timing.adc_read_ns, "ADC Read");
    print_pct(shell, timing.total_ns, timing.unset_strobe_ns, "Unset Strobe");
    print_pct(shell, timing.total_ns, timing.pull_drain_ns, "Pull Drain");
    print_pct(shell, timing.total_ns, timing.input_disconnect_ns, "Disconnect Input");

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS)

static int cmd_matrix_calibration_save(const struct shell *shell, size_t argc, char **argv,
                                       void *data) {
    struct matrix_hdl *matrix = get_matrix(argv[-2]);
    int ret = zmk_kscan_ec_matrix_settings_save_calibration(matrix->dev);
    if (ret < 0) {
        shell_print(shell, "Failed to initiate save calibration (%d)", ret);
    }

    return ret;
}

static int cmd_matrix_calibration_load(const struct shell *shell, size_t argc, char **argv,
                                       void *data) {
    struct matrix_hdl *matrix = get_matrix(argv[-2]);
    int ret = zmk_kscan_ec_matrix_settings_load_calibration(matrix->dev);
    if (ret < 0) {
        shell_print(shell, "Failed to initiate save calibration (%d)", ret);
    }

    return ret;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS)

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_matrix_calibration_cmds,
    /* Alphabetically sorted. */
    SHELL_CMD(start, NULL, CMD_HELP_CALIBRATION_START, cmd_matrix_calibration_start),
    SHELL_CMD(export, NULL, CMD_HELP_CALIBRATION_EXPORT, cmd_matrix_calibration_export),
#if IS_ENABLED(CONFIG_SETTINGS)
    SHELL_CMD(save, NULL, CMD_HELP_CALIBRATION_SAVE, cmd_matrix_calibration_save),
    SHELL_CMD(load, NULL, CMD_HELP_CALIBRATION_LOAD, cmd_matrix_calibration_load),
#endif                   // IS_ENABLED(CONFIG_SETTINGS)
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_matrix_cmds,
    /* Alphabetically sorted. */
    SHELL_CMD(calibration, &sub_matrix_calibration_cmds, CMD_HELP_CALIBRATE, NULL),
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
    SHELL_CMD(scan_rate, NULL, CMD_HELP_SCAN_RATE, cmd_matrix_scan_rate),
#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    SHELL_CMD(read_timing, NULL, CMD_HELP_READ_TIMING, cmd_matrix_read_timing),
#endif                   // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

static void cmd_matrix_dev_get(size_t idx, struct shell_static_entry *entry) {
    /* -1 because the last element in the list is a "list terminator" */
    if (idx < ARRAY_SIZE(matrix_hdl_list) - 1) {
        entry->syntax = matrix_hdl_list[idx].dev->name;
        entry->handler = NULL;
        entry->subcmd = &sub_matrix_cmds;
        entry->help = "Select subcommand for matrix property label.\n";
    } else {
        entry->syntax = NULL;
    }
}
SHELL_DYNAMIC_CMD_CREATE(sub_ec_matrix_dev, cmd_matrix_dev_get);

SHELL_CMD_REGISTER(ec, &sub_ec_matrix_dev, "EC Matrix commands", NULL);
