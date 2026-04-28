#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AXP2101_CHARGE_STATE_STANDBY = 0,
    AXP2101_CHARGE_STATE_CHARGING = 1,
    AXP2101_CHARGE_STATE_DISCHARGING = 2,
    AXP2101_CHARGE_STATE_UNKNOWN = 255,
} axp2101_charge_state_t;

typedef struct {
    bool chip_online;
    bool battery_connected;
    bool vbus_good;
    bool vbus_in;
    int battery_percent;
    int battery_voltage_mv;
    axp2101_charge_state_t charge_state;
    uint8_t raw_status1;
    uint8_t raw_status2;
} axp2101_pmu_telemetry_t;

esp_err_t axp2101_pmu_init(void);
esp_err_t axp2101_pmu_read(axp2101_pmu_telemetry_t *out);
const char *axp2101_charge_state_to_string(axp2101_charge_state_t state);

#ifdef __cplusplus
}
#endif
