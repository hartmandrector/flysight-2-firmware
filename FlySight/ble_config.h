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

#ifndef BLE_CONFIG_H_
#define BLE_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

/*
 * BLE Configuration and Divider Calculation
 *
 * This module provides divider calculation for BLE sensor streaming,
 * ensuring that sensor data transmission rates respect both hardware ODR
 * settings and BLE radio bandwidth constraints.
 */

/* BLE Safe Throughput Budget (bytes/sec) */
#define BLE_SAFE_THROUGHPUT_LIMIT  1500

/* Minimum BLE transmission rate per sensor (Hz) */
#define BLE_MIN_RATE_HZ            0.5

/* GPS packet size (bytes) */
#define BLE_GPS_PACKET_SIZE        44

/*
 * Calculate BLE divider for a sensor
 *
 * Returns divider value of 1 (no decimation). Auto-calculation is now
 * handled by FS_BLE_AutoCalculateDividers() with priority-based logic.
 *
 * Parameters:
 *   odr_setting - ODR configuration value (0-11)
 *   odr_table   - Pointer to ODR-to-Hz lookup table
 *   table_size  - Number of entries in lookup table
 *
 * Returns:
 *   Always returns 1 (no decimation)
 */
uint16_t FS_BLE_CalculateDivider(uint8_t odr_setting,
                                   const uint16_t *odr_table,
                                   uint8_t table_size);

/*
 * Calculate maximum divider for minimum rate
 *
 * Determines the maximum divider that keeps BLE rate ≥ BLE_MIN_RATE_HZ.
 *
 * Parameters:
 *   hw_rate_hz - Hardware ODR in Hz
 *
 * Returns:
 *   Maximum divider value (1-65535). Returns 1 if hw_rate_hz ≤ BLE_MIN_RATE_HZ.
 */
uint16_t FS_BLE_CalculateMaxDivider(uint16_t hw_rate_hz);

/*
 * Validation result structure
 */
typedef struct {
    bool     valid;                    // true if config is safe
    uint32_t estimated_bytes_per_sec;  // Total estimated BLE throughput
    char     error_msg[128];           // Error description if invalid
} FS_BLE_ValidationResult_t;

/*
 * Validate BLE configuration
 *
 * Calculates total expected BLE throughput based on GPS rate, sensor ODR
 * settings, and dividers. Checks against BLE_SAFE_THROUGHPUT_LIMIT.
 *
 * Parameters:
 *   config - Pointer to configuration structure
 *
 * Returns:
 *   Validation result with throughput estimate and error message if invalid
 *
 * Note: This function accounts for GPS rate (which has no divider) and all
 *       enabled sensors with their configured ODRs and dividers.
 */
FS_BLE_ValidationResult_t FS_BLE_ValidateConfig(const FS_Config_Data_t *config);

/*
 * Auto-calculate BLE dividers with priority-based allocation
 *
 * Priority order:
 *   1. GPS rate (no divider, controlled by Rate parameter)
 *   2. Manual sensor dividers (user-specified, non-zero values)
 *   3. Auto sensor dividers (divider=0, calculated to fit budget)
 *
 * Algorithm:
 *   1. Calculate GPS + manual divider bandwidth
 *   2. Try auto sensors at full ODR (divider=1)
 *   3. If over budget, scale auto dividers by percentage to fit
 *   4. Enforce minimum rate (BLE_MIN_RATE_HZ) on all sensors
 *
 * This respects user ODR choices - sensors run at full rate unless
 * bandwidth requires throttling. Percentage-based scaling maintains
 * relative priorities (higher ODR sensors stay higher after scaling).
 *
 * Parameters:
 *   config - Pointer to configuration structure (modified in place)
 *
 * Note: Only modifies dividers that are set to 0. User-specified dividers
 *       are left unchanged. Call this after parsing config but before
 *       validation.
 */
void FS_BLE_AutoCalculateDividers(FS_Config_Data_t *config);

#endif /* BLE_CONFIG_H_ */
