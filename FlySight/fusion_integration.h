/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2024 Bionic Avionics Inc.                                   **
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

#ifndef FUSION_INTEGRATION_H_
#define FUSION_INTEGRATION_H_

#include <stdint.h>
#include <stdbool.h>
#include "Fusion.h"

/**
 * Fusion output data structure
 * All angles in degrees for easy BLE transmission
 */
typedef struct {
    uint32_t time;          // ms timestamp
    float q_w;              // Quaternion W component
    float q_x;              // Quaternion X component  
    float q_y;              // Quaternion Y component
    float q_z;              // Quaternion Z component
    float heading;          // Magnetic heading (0-360 degrees)
    float pitch;            // Pitch angle (degrees)
    float roll;             // Roll angle (degrees)
    bool initialising;      // True during initial convergence (~3 seconds)
} FS_Fusion_Data_t;

/**
 * Fusion internal states for diagnostics
 */
typedef struct {
    float accelerationError;    // Accel error in degrees
    float magneticError;        // Mag error in degrees
    bool accelerometerIgnored;  // True if accel rejected (high-G)
    bool magnetometerIgnored;   // True if mag rejected (distortion)
    bool angularRateRecovery;   // True if gyro range exceeded
} FS_Fusion_Status_t;

/**
 * Initialize the sensor fusion module
 * Reads calibration from config if available
 */
void FS_Fusion_Init(void);

/**
 * Start sensor fusion processing
 */
void FS_Fusion_Start(void);

/**
 * Stop sensor fusion processing
 */
void FS_Fusion_Stop(void);

/**
 * Process new magnetometer data
 * Called from FS_Mag_DataReady_Callback at ~100 Hz
 * 
 * @param x Magnetometer X (gauss * 1000)
 * @param y Magnetometer Y (gauss * 1000)
 * @param z Magnetometer Z (gauss * 1000)
 */
void FS_Fusion_UpdateMag(int16_t x, int16_t y, int16_t z);

/**
 * Process new IMU data and run fusion algorithm
 * Called from FS_IMU_DataReady_Callback at ~416 Hz
 * This is the main fusion update - gyro integration happens here
 * 
 * @param time_ms  Timestamp in milliseconds
 * @param wx       Gyro X (deg/s * 1000)
 * @param wy       Gyro Y (deg/s * 1000)
 * @param wz       Gyro Z (deg/s * 1000)
 * @param ax       Accel X (g * 100000)
 * @param ay       Accel Y (g * 100000)
 * @param az       Accel Z (g * 100000)
 */
void FS_Fusion_UpdateIMU(uint32_t time_ms,
                         int32_t wx, int32_t wy, int32_t wz,
                         int32_t ax, int32_t ay, int32_t az);

/**
 * Get current fusion output
 * @return Pointer to fusion data structure
 */
const FS_Fusion_Data_t* FS_Fusion_GetData(void);

/**
 * Get current quaternion directly
 * @param q Array of 4 floats to fill [w, x, y, z]
 */
void FS_Fusion_GetQuaternion(float q[4]);

/**
 * Get fusion diagnostic status
 * @return Pointer to status structure
 */
const FS_Fusion_Status_t* FS_Fusion_GetStatus(void);

/**
 * Check if fusion is producing valid output
 * @return true if fusion has converged (after ~3 seconds)
 */
bool FS_Fusion_IsValid(void);

/**
 * Set magnetometer hard-iron calibration at runtime
 * @param offset Hard iron offset vector (gauss)
 */
void FS_Fusion_SetMagHardIron(FusionVector offset);

/**
 * Set magnetometer soft-iron calibration at runtime
 * @param matrix 3x3 soft-iron correction matrix
 */
void FS_Fusion_SetMagSoftIron(FusionMatrix matrix);

/**
 * Get linear acceleration (gravity removed) in body frame
 * @return Linear acceleration in g
 */
FusionVector FS_Fusion_GetLinearAccel(void);

/**
 * Get linear acceleration in Earth frame (NWU)
 * @return Earth-frame acceleration in g
 */
FusionVector FS_Fusion_GetEarthAccel(void);

#endif /* FUSION_INTEGRATION_H_ */
