/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "ble_config.h"
#include "sensor_odr.h"

uint16_t FS_BLE_CalculateDivider(uint8_t odr_setting,
                                   const uint16_t *odr_table,
                                   uint8_t table_size)
{
    // Validate inputs
    if (odr_table == NULL || odr_setting >= table_size) {
        return 1;  // Default to no decimation on error
    }
    
    // Get hardware rate from ODR setting
    uint16_t hw_rate_hz = odr_table[odr_setting];
    
    // If hardware rate is 0 (power-down) or already ≤ target rate, no division needed
    if (hw_rate_hz == 0 || hw_rate_hz <= BLE_TARGET_RATE_HZ) {
        return 1;
    }
    
    // Calculate minimum divider to bring rate to ≤ target Hz
    // Use ceiling division to ensure we don't exceed target
    uint16_t divider = (hw_rate_hz + BLE_TARGET_RATE_HZ - 1) / BLE_TARGET_RATE_HZ;
    
    return divider;
}

FS_BLE_ValidationResult_t FS_BLE_ValidateConfig(const FS_Config_Data_t *config)
{
    FS_BLE_ValidationResult_t result;
    memset(&result, 0, sizeof(result));
    
    uint32_t total = 0;
    
    // GPS rate (no divider - user controlled via Rate parameter)
    if (config->rate > 0 && config->enable_gnss) {
        uint16_t gps_hz = 1000 / config->rate;  // Convert ms to Hz
        total += (gps_hz * BLE_GPS_PACKET_SIZE);
    }
    
    // Helper macro for each sensor
    #define CHECK_SENSOR(enable, odr_field, div_field, odr_table, packet_size) \
        do { \
            if (config->enable) { \
                uint16_t hz = ODR_TO_HZ(odr_table, sizeof(odr_table)/sizeof(odr_table[0]), config->odr_field); \
                uint16_t div = (config->div_field == 0) ? \
                    FS_BLE_CalculateDivider(config->odr_field, odr_table, sizeof(odr_table)/sizeof(odr_table[0])) : \
                    config->div_field; \
                if (hz > 0 && div > 0) { \
                    uint16_t ble_hz = hz / div; \
                    total += (ble_hz * packet_size); \
                } \
            } \
        } while(0)
    
    CHECK_SENSOR(enable_baro, baro_odr, ble_baro_divider, baro_odr_table, 11);
    CHECK_SENSOR(enable_hum, hum_odr, ble_hum_divider, hum_odr_table, 9);
    CHECK_SENSOR(enable_imu, accel_odr, ble_accel_divider, accel_odr_table, 19);
    CHECK_SENSOR(enable_imu, gyro_odr, ble_gyro_divider, gyro_odr_table, 19);
    CHECK_SENSOR(enable_mag, mag_odr, ble_mag_divider, mag_odr_table, 13);
    
    #undef CHECK_SENSOR
    
    result.estimated_bytes_per_sec = total;
    
    if (total > BLE_SAFE_THROUGHPUT_LIMIT) {
        result.valid = false;
        snprintf(result.error_msg, sizeof(result.error_msg),
                 "BLE bandwidth exceeded: %lu bytes/sec (limit %d bytes/sec). "
                 "Reduce GPS rate, sensor ODRs, or increase BLE dividers.",
                 (unsigned long)total, BLE_SAFE_THROUGHPUT_LIMIT);
    } else {
        result.valid = true;
        snprintf(result.error_msg, sizeof(result.error_msg),
                 "BLE config valid: %lu bytes/sec (%.0f%% of limit)",
                 (unsigned long)total, (100.0f * total) / BLE_SAFE_THROUGHPUT_LIMIT);
    }
    
    return result;
}
