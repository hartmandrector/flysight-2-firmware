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

/* Target BLE transmission rate per sensor (Hz) */
#define BLE_TARGET_RATE_HZ         15

/* GPS packet size (bytes) */
#define BLE_GPS_PACKET_SIZE        44

/*
 * Calculate BLE divider for a sensor
 *
 * Determines the appropriate decimation divider to achieve ≤ 15Hz BLE
 * transmission rate for a sensor, regardless of its hardware ODR.
 *
 * Parameters:
 *   odr_setting - ODR configuration value (0-11)
 *   odr_table   - Pointer to ODR-to-Hz lookup table
 *   table_size  - Number of entries in lookup table
 *
 * Returns:
 *   Divider value (1-65535). Returns 1 if hardware rate ≤ 15Hz or disabled.
 *
 * Examples:
 *   12.5Hz → divider 1 (no decimation)
 *   104Hz  → divider 7 (→ 14.9Hz BLE)
 *   416Hz  → divider 28 (→ 14.9Hz BLE)
 *   6666Hz → divider 445 (→ 15Hz BLE)
 */
uint16_t FS_BLE_CalculateDivider(uint8_t odr_setting,
                                   const uint16_t *odr_table,
                                   uint8_t table_size);

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

#endif /* BLE_CONFIG_H_ */
