
#pragma once

#include <zephyr/device.h>

enum adxl362_awake_trigger_activity_limit {
    ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_NORMAL = 0,
    ADXL362_AWAKE_TRIGGER_ACTIVITY_LIMIT_SLEEP = 1,
};

int adxl362_awake_trigger_set_activity_limit(const struct device *dev, enum adxl362_awake_trigger_activity_limit limit);