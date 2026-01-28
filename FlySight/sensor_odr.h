/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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

#ifndef SENSOR_ODR_H_
#define SENSOR_ODR_H_

#include <stdint.h>

/*
 * ODR to Hz Lookup Tables
 *
 * These tables convert sensor Output Data Rate (ODR) configuration values
 * (0-11) to actual hardware sampling rates in Hz.
 *
 * Used by BLE divider calculation algorithm to determine appropriate
 * decimation for Bluetooth transmission.
 */

/* Barometer (LPS22HH) - ODR values 0-7 */
static const uint16_t baro_odr_table[] = {
    0,     // 0: Power-down / One-shot
    1,     // 1: 1 Hz
    10,    // 2: 10 Hz
    25,    // 3: 25 Hz
    50,    // 4: 50 Hz
    75,    // 5: 75 Hz
    100,   // 6: 100 Hz
    200    // 7: 200 Hz (low-noise mode)
};

/* Humidity (HTS221/SHT4x) - ODR values 0-3 */
static const uint16_t hum_odr_table[] = {
    0,     // 0: Power-down / One-shot
    1,     // 1: 1 Hz
    7,     // 2: 7 Hz (HTS221 only)
    12     // 3: 12.5 Hz → 12 (HTS221 maximum)
};

/* Magnetometer (LIS2MDL) - ODR values 0-3 */
static const uint16_t mag_odr_table[] = {
    10,    // 0: 10 Hz
    20,    // 1: 20 Hz
    50,    // 2: 50 Hz
    100    // 3: 100 Hz (maximum)
};

/* Accelerometer (LSM6DSO) - ODR values 0-11 */
static const uint16_t accel_odr_table[] = {
    0,     // 0:  Power-down
    12,    // 1:  12.5 Hz → 12 (low-power)
    26,    // 2:  26 Hz (low-power)
    52,    // 3:  52 Hz (low-power)
    104,   // 4:  104 Hz (normal)
    208,   // 5:  208 Hz (normal)
    416,   // 6:  416 Hz (high-performance)
    833,   // 7:  833 Hz (high-performance)
    1666,  // 8:  1666 Hz (high-performance)
    3333,  // 9:  3333 Hz (high-performance)
    6666,  // 10: 6666 Hz (high-performance, maximum)
    2      // 11: 1.6 Hz → 2 (ultra-low-power)
};

/* Gyroscope (LSM6DSO) - ODR values 0-10 */
/* Note: Gyro does not support ODR value 11 (1.6 Hz) */
static const uint16_t gyro_odr_table[] = {
    0,     // 0:  Power-down
    12,    // 1:  12.5 Hz → 12 (low-power)
    26,    // 2:  26 Hz (low-power)
    52,    // 3:  52 Hz (low-power)
    104,   // 4:  104 Hz (normal)
    208,   // 5:  208 Hz (normal)
    416,   // 6:  416 Hz (high-performance)
    833,   // 7:  833 Hz (high-performance)
    1666,  // 8:  1666 Hz (high-performance)
    3333,  // 9:  3333 Hz (high-performance)
    6666   // 10: 6666 Hz (high-performance, maximum)
};

/* Table sizes for validation */
#define BARO_ODR_TABLE_SIZE   (sizeof(baro_odr_table) / sizeof(baro_odr_table[0]))
#define HUM_ODR_TABLE_SIZE    (sizeof(hum_odr_table) / sizeof(hum_odr_table[0]))
#define MAG_ODR_TABLE_SIZE    (sizeof(mag_odr_table) / sizeof(mag_odr_table[0]))
#define ACCEL_ODR_TABLE_SIZE  (sizeof(accel_odr_table) / sizeof(accel_odr_table[0]))
#define GYRO_ODR_TABLE_SIZE   (sizeof(gyro_odr_table) / sizeof(gyro_odr_table[0]))

/*
 * Helper macro to get Hz from ODR value safely
 * Returns 0 if odr is out of bounds for the table
 */
#define ODR_TO_HZ(table, table_size, odr) \
    (((odr) < (table_size)) ? (table)[(odr)] : 0)

#endif /* SENSOR_ODR_H_ */
