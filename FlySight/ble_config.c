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

void FS_BLE_AutoCalculateDividers(FS_Config_Data_t *config)
{
    // Step 1: Calculate GPS bandwidth
    uint32_t gps_bandwidth = 0;
    if (config->rate > 0 && config->enable_gnss) {
        uint16_t gps_hz = 1000 / config->rate;
        gps_bandwidth = gps_hz * BLE_GPS_PACKET_SIZE;
    }
    
    // Step 2: Calculate remaining budget for sensors
    int32_t sensor_budget = BLE_SAFE_THROUGHPUT_LIMIT - gps_bandwidth;
    if (sensor_budget < 100) {
        sensor_budget = 100;  // Minimum budget for at least some sensor data
    }
    
    // Step 3: Calculate initial dividers (target 15Hz per sensor) and estimate bandwidth
    typedef struct {
        bool enabled;
        uint16_t *divider_ptr;  // Pointer to config field
        uint8_t odr_setting;
        const uint16_t *odr_table;
        uint8_t table_size;
        uint16_t packet_size;
        uint16_t initial_divider;
        uint32_t bandwidth;  // At initial divider
    } sensor_calc_t;
    
    sensor_calc_t sensors[] = {
        { config->enable_baro, &config->ble_baro_divider, config->baro_odr, 
          baro_odr_table, sizeof(baro_odr_table)/sizeof(baro_odr_table[0]), 11, 0, 0 },
        { config->enable_hum, &config->ble_hum_divider, config->hum_odr,
          hum_odr_table, sizeof(hum_odr_table)/sizeof(hum_odr_table[0]), 9, 0, 0 },
        { config->enable_imu, &config->ble_accel_divider, config->accel_odr,
          accel_odr_table, sizeof(accel_odr_table)/sizeof(accel_odr_table[0]), 19, 0, 0 },
        { config->enable_imu, &config->ble_gyro_divider, config->gyro_odr,
          gyro_odr_table, sizeof(gyro_odr_table)/sizeof(gyro_odr_table[0]), 19, 0, 0 },
        { config->enable_mag, &config->ble_mag_divider, config->mag_odr,
          mag_odr_table, sizeof(mag_odr_table)/sizeof(mag_odr_table[0]), 13, 0, 0 },
    };
    
    uint32_t total_sensor_bandwidth = 0;
    int num_sensors = sizeof(sensors) / sizeof(sensors[0]);
    
    for (int i = 0; i < num_sensors; i++) {
        sensor_calc_t *s = &sensors[i];
        
        // Skip if sensor disabled or divider already set by user
        if (!s->enabled || *(s->divider_ptr) != 0) {
            continue;
        }
        
        // Calculate initial divider (target 15Hz)
        s->initial_divider = FS_BLE_CalculateDivider(s->odr_setting, s->odr_table, s->table_size);
        
        // Calculate bandwidth at this divider
        uint16_t hw_hz = ODR_TO_HZ(s->odr_table, s->table_size, s->odr_setting);
        if (hw_hz > 0 && s->initial_divider > 0) {
            uint16_t ble_hz = hw_hz / s->initial_divider;
            s->bandwidth = ble_hz * s->packet_size;
            total_sensor_bandwidth += s->bandwidth;
        }
    }
    
    // Step 4: If total exceeds sensor budget, scale all dividers proportionally
    if (total_sensor_bandwidth > (uint32_t)sensor_budget) {
        // Calculate scale factor (how much we need to reduce bandwidth)
        // scale > 1.0 means we need to increase dividers
        float scale = (float)total_sensor_bandwidth / (float)sensor_budget;
        
        // Apply scale to all auto-calculated dividers
        for (int i = 0; i < num_sensors; i++) {
            sensor_calc_t *s = &sensors[i];
            
            if (!s->enabled || s->initial_divider == 0) {
                continue;
            }
            
            // Only scale if this sensor's divider is in auto mode (was 0)
            if (*(s->divider_ptr) == 0) {
                // Scale up the divider (ceiling to ensure we stay under budget)
                uint16_t scaled_divider = (uint16_t)(s->initial_divider * scale + 0.5f);
                if (scaled_divider < 1) scaled_divider = 1;
                *(s->divider_ptr) = scaled_divider;
            }
        }
    } else {
        // Step 5: Budget not exceeded - just use initial dividers
        for (int i = 0; i < num_sensors; i++) {
            sensor_calc_t *s = &sensors[i];
            
            if (s->enabled && s->initial_divider > 0 && *(s->divider_ptr) == 0) {
                *(s->divider_ptr) = s->initial_divider;
            }
        }
    }
}
