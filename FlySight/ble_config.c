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
    // Auto-calculation now handled by FS_BLE_AutoCalculateDividers()
    // This function just returns 1 (no decimation) for compatibility
    (void)odr_setting;
    (void)odr_table;
    (void)table_size;
    return 1;
}

uint16_t FS_BLE_CalculateMaxDivider(uint16_t hw_rate_hz)
{
    // Calculate maximum divider that keeps BLE rate ≥ BLE_MIN_RATE_HZ
    if (hw_rate_hz == 0) {
        return 1;  // Power-down or invalid
    }
    
    // max_divider = hw_rate_hz / BLE_MIN_RATE_HZ
    // Use 0.5 Hz minimum to allow integer math: max_div = hw_rate * 2
    uint16_t max_divider = hw_rate_hz * 2;  // For 0.5 Hz minimum
    
    if (max_divider < 1) {
        max_divider = 1;
    }
    
    return max_divider;
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
    // If divider is 0, assume divider=1 (will be auto-calculated later)
    #define CHECK_SENSOR(enable, odr_field, div_field, odr_table, packet_size) \
        do { \
            if (config->enable) { \
                uint16_t hz = ODR_TO_HZ(odr_table, sizeof(odr_table)/sizeof(odr_table[0]), config->odr_field); \
                uint16_t div = (config->div_field == 0) ? 1 : config->div_field; \
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
    // Priority-based algorithm:
    // 1. GPS + manual dividers take priority (calculate their bandwidth)
    // 2. Try auto sensors at full ODR (divider=1)
    // 3. If over budget, scale auto dividers by percentage to fit
    // 4. Enforce minimum rate on all sensors
    
    // Step 1: Calculate GPS bandwidth (highest priority)
    uint32_t gps_bandwidth = 0;
    if (config->rate > 0 && config->enable_gnss) {
        uint16_t gps_hz = 1000 / config->rate;
        gps_bandwidth = gps_hz * BLE_GPS_PACKET_SIZE;
    }
    
    // Define sensor array for calculations
    typedef struct {
        bool enabled;
        uint16_t *divider_ptr;  // Pointer to config field
        uint8_t odr_setting;
        const uint16_t *odr_table;
        uint8_t table_size;
        uint16_t packet_size;
        uint16_t hw_hz;          // Hardware ODR in Hz
        uint32_t bandwidth;      // Calculated bandwidth
        bool is_manual;          // true if user set non-zero divider
    } sensor_calc_t;
    
    sensor_calc_t sensors[] = {
        { config->enable_baro, &config->ble_baro_divider, config->baro_odr, 
          baro_odr_table, sizeof(baro_odr_table)/sizeof(baro_odr_table[0]), 11, 0, 0, false },
        { config->enable_hum, &config->ble_hum_divider, config->hum_odr,
          hum_odr_table, sizeof(hum_odr_table)/sizeof(hum_odr_table[0]), 9, 0, 0, false },
        { config->enable_imu, &config->ble_accel_divider, config->accel_odr,
          accel_odr_table, sizeof(accel_odr_table)/sizeof(accel_odr_table[0]), 19, 0, 0, false },
                { config->enable_imu, &config->ble_gyro_divider, config->gyro_odr,
                    gyro_odr_table, sizeof(gyro_odr_table)/sizeof(gyro_odr_table[0]), 19, 0, 0, false },
        { config->enable_mag, &config->ble_mag_divider, config->mag_odr,
          mag_odr_table, sizeof(mag_odr_table)/sizeof(mag_odr_table[0]), 13, 0, 0, false },
    };
    
    int num_sensors = sizeof(sensors) / sizeof(sensors[0]);
    
    // Get hardware rates and identify manual vs auto sensors
    for (int i = 0; i < num_sensors; i++) {
        sensor_calc_t *s = &sensors[i];
        if (!s->enabled) {
            continue;
        }
        
        s->hw_hz = ODR_TO_HZ(s->odr_table, s->table_size, s->odr_setting);
        s->is_manual = (*(s->divider_ptr) != 0);
    }
    
    // Step 2: Calculate manual divider bandwidth (second priority)
    uint32_t manual_bandwidth = 0;
    for (int i = 0; i < num_sensors; i++) {
        sensor_calc_t *s = &sensors[i];
        if (!s->enabled || !s->is_manual || s->hw_hz == 0) {
            continue;
        }
        
        uint16_t ble_hz = s->hw_hz / *(s->divider_ptr);
        s->bandwidth = ble_hz * s->packet_size;
        manual_bandwidth += s->bandwidth;
    }
    
    // Step 3: Calculate remaining budget for auto sensors
    int32_t remaining_budget = BLE_SAFE_THROUGHPUT_LIMIT - gps_bandwidth - manual_bandwidth;
    if (remaining_budget < 0) {
        remaining_budget = 0;  // Manual + GPS already exceed budget
    }
    
    // Step 4: Try auto sensors at full ODR (divider=1)
    uint32_t auto_bandwidth = 0;
    for (int i = 0; i < num_sensors; i++) {
        sensor_calc_t *s = &sensors[i];
        if (!s->enabled || s->is_manual || s->hw_hz == 0) {
            continue;
        }
        
        // Try divider=1 (full ODR)
        uint16_t ble_hz = s->hw_hz / 1;
        s->bandwidth = ble_hz * s->packet_size;
        auto_bandwidth += s->bandwidth;
    }
    
    // Step 5: If auto sensors exceed remaining budget, scale by percentage
    if (auto_bandwidth > (uint32_t)remaining_budget && remaining_budget > 0) {
        // Calculate scale factor (percentage we need to reduce bandwidth)
        float scale = (float)auto_bandwidth / (float)remaining_budget;
        
        // Apply percentage-based scaling to all auto sensors
        for (int i = 0; i < num_sensors; i++) {
            sensor_calc_t *s = &sensors[i];
            if (!s->enabled || s->is_manual || s->hw_hz == 0) {
                continue;
            }
            
            // Scale divider up by percentage
            // new_divider = 1 * scale
            uint16_t scaled_divider = (uint16_t)(1.0f * scale + 0.5f);
            if (scaled_divider < 1) {
                scaled_divider = 1;
            }
            
            // Enforce minimum rate (0.5 Hz)
            uint16_t max_divider = FS_BLE_CalculateMaxDivider(s->hw_hz);
            if (scaled_divider > max_divider) {
                scaled_divider = max_divider;
            }
            
            *(s->divider_ptr) = scaled_divider;
        }
    } else if (remaining_budget == 0) {
        // Manual + GPS already exceeded budget - set auto sensors to minimum rate
        for (int i = 0; i < num_sensors; i++) {
            sensor_calc_t *s = &sensors[i];
            if (!s->enabled || s->is_manual || s->hw_hz == 0) {
                continue;
            }
            
            // Set to minimum rate (0.5 Hz)
            uint16_t max_divider = FS_BLE_CalculateMaxDivider(s->hw_hz);
            *(s->divider_ptr) = max_divider;
        }
    } else {
        // Auto sensors fit within budget - use divider=1 (full ODR)
        for (int i = 0; i < num_sensors; i++) {
            sensor_calc_t *s = &sensors[i];
            if (!s->enabled || s->is_manual || s->hw_hz == 0) {
                continue;
            }
            
            *(s->divider_ptr) = 1;
        }
    }
}
