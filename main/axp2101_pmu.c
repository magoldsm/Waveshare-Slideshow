#include "axp2101_pmu.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "bsp/esp-bsp.h"

#define AXP2101_I2C_ADDR 0x34

#define AXP2101_REG_STATUS1 0x00
#define AXP2101_REG_STATUS2 0x01
#define AXP2101_REG_CHIP_ID 0x03
#define AXP2101_REG_BATT_VOLT_H 0x34
#define AXP2101_REG_BATT_VOLT_L 0x35
#define AXP2101_REG_BATT_PERCENT 0xA4

#define AXP2101_I2C_SPEED_HZ 400000
#define AXP2101_TIMEOUT_MS 100

static i2c_master_dev_handle_t s_axp_dev = NULL;
static bool s_axp_initialized = false;

static esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value)
{
    if (s_axp_dev == NULL || value == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(s_axp_dev,
                                       &reg,
                                       1,
                                       value,
                                       1,
                                       AXP2101_TIMEOUT_MS);
}

esp_err_t axp2101_pmu_init(void)
{
    if (s_axp_initialized && s_axp_dev != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t i2c_bus = bsp_i2c_get_handle();
    if (i2c_bus == NULL) {
        esp_err_t err = bsp_i2c_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        i2c_bus = bsp_i2c_get_handle();
    }

    if (i2c_bus == NULL) {
        return ESP_FAIL;
    }

    if (s_axp_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = AXP2101_I2C_ADDR,
            .scl_speed_hz = AXP2101_I2C_SPEED_HZ,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = 0,
            },
        };

        esp_err_t add_err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &s_axp_dev);
        if (add_err != ESP_OK) {
            return add_err;
        }
    }

    uint8_t chip_id = 0;
    esp_err_t read_err = axp2101_read_reg(AXP2101_REG_CHIP_ID, &chip_id);
    if (read_err != ESP_OK) {
        return read_err;
    }

    s_axp_initialized = true;
    return ESP_OK;
}

esp_err_t axp2101_pmu_read(axp2101_pmu_telemetry_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->battery_percent = -1;
    out->battery_voltage_mv = -1;
    out->charge_state = AXP2101_CHARGE_STATE_UNKNOWN;

    esp_err_t init_err = axp2101_pmu_init();
    if (init_err != ESP_OK) {
        return init_err;
    }

    uint8_t status1 = 0;
    uint8_t status2 = 0;

    esp_err_t err = axp2101_read_reg(AXP2101_REG_STATUS1, &status1);
    if (err != ESP_OK) {
        return err;
    }

    err = axp2101_read_reg(AXP2101_REG_STATUS2, &status2);
    if (err != ESP_OK) {
        return err;
    }

    out->chip_online = true;
    out->raw_status1 = status1;
    out->raw_status2 = status2;

    out->vbus_good = ((status1 >> 5) & 0x01) != 0;
    out->battery_connected = ((status1 >> 3) & 0x01) != 0;
    out->vbus_in = (((status2 >> 3) & 0x01) == 0) && out->vbus_good;

    uint8_t charge_bits = (status2 >> 5) & 0x03;
    if (charge_bits == 0) {
        out->charge_state = AXP2101_CHARGE_STATE_STANDBY;
    } else if (charge_bits == 1) {
        out->charge_state = AXP2101_CHARGE_STATE_CHARGING;
    } else if (charge_bits == 2) {
        out->charge_state = AXP2101_CHARGE_STATE_DISCHARGING;
    } else {
        out->charge_state = AXP2101_CHARGE_STATE_UNKNOWN;
    }

    if (out->battery_connected) {
        uint8_t batt_percent = 0;
        if (axp2101_read_reg(AXP2101_REG_BATT_PERCENT, &batt_percent) == ESP_OK && batt_percent <= 100) {
            out->battery_percent = (int)batt_percent;
        }

        uint8_t batt_h = 0;
        uint8_t batt_l = 0;
        if (axp2101_read_reg(AXP2101_REG_BATT_VOLT_H, &batt_h) == ESP_OK &&
            axp2101_read_reg(AXP2101_REG_BATT_VOLT_L, &batt_l) == ESP_OK) {
            // XPowers uses readRegisterH5L8 for AXP2101 battery voltage.
            out->battery_voltage_mv = (int)(((uint16_t)(batt_h & 0x1F) << 8) | batt_l);
        }
    }

    return ESP_OK;
}

const char *axp2101_charge_state_to_string(axp2101_charge_state_t state)
{
    switch (state) {
    case AXP2101_CHARGE_STATE_STANDBY:
        return "standby";
    case AXP2101_CHARGE_STATE_CHARGING:
        return "charging";
    case AXP2101_CHARGE_STATE_DISCHARGING:
        return "discharging";
    default:
        return "unknown";
    }
}
