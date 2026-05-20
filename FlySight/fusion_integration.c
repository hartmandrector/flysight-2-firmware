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

/**
 * Sensor Fusion Integration for FlySight 2
 * 
 * Uses the x-io Fusion library (Chapter 7 AHRS algorithm) with:
 * - Acceleration rejection for high-G maneuvers
 * - Magnetic distortion rejection  
 * - Gyroscope bias estimation (FusionBias)
 * - Coordinate frame remapping for LIS2MDL magnetometer
 * 
 * Data flow:
 *   MAG (~100 Hz) --> FS_Fusion_UpdateMag() --> stores calibrated mag vector
 *   IMU (~416 Hz) --> FS_Fusion_UpdateIMU() --> runs AHRS fusion, updates quaternion
 * 
 * The magnetometer is on the BACK of the PCB, requiring axis remapping:
 *   Device X = -Sensor X
 *   Device Y = +Sensor Y  
 *   Device Z = -Sensor Z
 */

#include "fusion_integration.h"
#include "config.h"
#include "log.h"

/* Unit conversion constants */
#define GYRO_SCALE    (1.0f / 1000.0f)       /* deg/s*1000 -> deg/s */
#define ACCEL_SCALE   (1.0f / 100000.0f)     /* g*100000 -> g */
#define MAG_SCALE     (1.0f / 1000.0f)       /* gauss*1000 -> gauss */

/* Sample rate for gyro bias estimation */
#define SAMPLE_RATE_HZ  416

/**
 * Get gyroscope full-scale range based on config setting
 * @param gyro_fs Config value (0-3)
 * @return Range in degrees per second
 */
static inline float GetGyroFullScale(uint8_t gyro_fs)
{
    switch (gyro_fs)
    {
    case 0: return 250.0f;   // GYRO_FS_250
    case 1: return 500.0f;   // GYRO_FS_500
    case 2: return 1000.0f;  // GYRO_FS_1000
    case 3: return 2000.0f;  // GYRO_FS_2000
    default: return 2000.0f; // Default to max range
    }
}

/* State */
static bool fusion_enabled = false;
static bool fusion_active = false;
static uint32_t last_imu_time = 0;
static FS_Fusion_Data_t fusion_data;
static FS_Fusion_Status_t fusion_status;

/* x-io Fusion structures */
static FusionAhrs ahrs;
static FusionBias bias;

/* Magnetometer calibration */
static FusionVector mag_hard_iron = FUSION_VECTOR_ZERO;
static FusionMatrix mag_soft_iron = FUSION_MATRIX_IDENTITY;

/* IMU calibration */
static FusionVector gyro_bias = FUSION_VECTOR_ZERO;
static FusionMatrix accel_matrix = FUSION_MATRIX_IDENTITY;

/* Last magnetometer reading (async from IMU) */
static FusionVector last_mag = FUSION_VECTOR_ZERO;
static bool mag_valid = false;

void FS_Fusion_Init(void)
{
    const FS_Config_Data_t *cfg = FS_Config_Get();
    
    /* Check if fusion is enabled */
    fusion_enabled = cfg->enable_fusion;
    if (!fusion_enabled) return;
    
    /* Initialize gyroscope bias estimation */
    FusionBiasInitialise(&bias, SAMPLE_RATE_HZ);
    
    /* Initialize AHRS algorithm */
    FusionAhrsInitialise(&ahrs);
    
    /* Configure AHRS settings from config file */
    FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,      /* North-West-Up */
        .gain = (float)cfg->fusion_gain / 100.0f,
        .gyroscopeRange = GetGyroFullScale(cfg->gyro_fs),  /* Actual configured range */
        .accelerationRejection = (float)cfg->fusion_accel_reject,
        .magneticRejection = (float)cfg->fusion_mag_reject,
        .recoveryTriggerPeriod = SAMPLE_RATE_HZ * cfg->fusion_timeout,  /* seconds -> samples */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    
    /* Load magnetometer calibration from config (milligauss -> gauss) */
    mag_hard_iron = (FusionVector){ .axis = {
        (float)cfg->fusion_mag_hard_x / 1000.0f,
        (float)cfg->fusion_mag_hard_y / 1000.0f,
        (float)cfg->fusion_mag_hard_z / 1000.0f
    }};
    
    /* Load magnetometer soft iron matrix from config (scaled by 1000000) */
    mag_soft_iron = (FusionMatrix){ .element = {
        .xx = (float)cfg->fusion_mag_soft_m[0] / 1000000.0f,
        .xy = (float)cfg->fusion_mag_soft_m[1] / 1000000.0f,
        .xz = (float)cfg->fusion_mag_soft_m[2] / 1000000.0f,
        .yx = (float)cfg->fusion_mag_soft_m[3] / 1000000.0f,
        .yy = (float)cfg->fusion_mag_soft_m[4] / 1000000.0f,
        .yz = (float)cfg->fusion_mag_soft_m[5] / 1000000.0f,
        .zx = (float)cfg->fusion_mag_soft_m[6] / 1000000.0f,
        .zy = (float)cfg->fusion_mag_soft_m[7] / 1000000.0f,
        .zz = (float)cfg->fusion_mag_soft_m[8] / 1000000.0f,
    }};
    
    /* Load gyro bias from config (scaled by 10000 -> deg/s) */
    gyro_bias = (FusionVector){ .axis = {
        (float)cfg->fusion_gyro_bias_x / 10000.0f,
        (float)cfg->fusion_gyro_bias_y / 10000.0f,
        (float)cfg->fusion_gyro_bias_z / 10000.0f
    }};
    
    /* Load accelerometer scale matrix from config (scaled by 1000000) */
    accel_matrix = (FusionMatrix){ .element = {
        .xx = (float)cfg->fusion_accel_m[0] / 1000000.0f,
        .xy = (float)cfg->fusion_accel_m[1] / 1000000.0f,
        .xz = (float)cfg->fusion_accel_m[2] / 1000000.0f,
        .yx = (float)cfg->fusion_accel_m[3] / 1000000.0f,
        .yy = (float)cfg->fusion_accel_m[4] / 1000000.0f,
        .yz = (float)cfg->fusion_accel_m[5] / 1000000.0f,
        .zx = (float)cfg->fusion_accel_m[6] / 1000000.0f,
        .zy = (float)cfg->fusion_accel_m[7] / 1000000.0f,
        .zz = (float)cfg->fusion_accel_m[8] / 1000000.0f,
    }};
    
    /* Reset state */
    last_imu_time = 0;
    mag_valid = false;
    
    /* Clear output */
    fusion_data.time = 0;
    fusion_data.q_w = 1.0f;
    fusion_data.q_x = 0.0f;
    fusion_data.q_y = 0.0f;
    fusion_data.q_z = 0.0f;
    fusion_data.heading = 0.0f;
    fusion_data.pitch = 0.0f;
    fusion_data.roll = 0.0f;
    fusion_data.initialising = true;
}

void FS_Fusion_Start(void)
{
    if (!fusion_enabled) return;
    
    FusionAhrsReset(&ahrs);
    FusionBiasInitialise(&bias, SAMPLE_RATE_HZ);
    fusion_active = true;
    last_imu_time = 0;
    mag_valid = false;
}

void FS_Fusion_Stop(void)
{
    fusion_active = false;
}

void FS_Fusion_UpdateMag(int16_t x, int16_t y, int16_t z)
{
    if (!fusion_enabled || !fusion_active) return;
    
    /* Convert from integer (gauss*1000) to float (gauss) */
    FusionVector raw_mag = {
        .axis = {
            .x = (float)x * MAG_SCALE,
            .y = (float)y * MAG_SCALE,
            .z = (float)z * MAG_SCALE,
        }
    };
    
    /* 
     * Order of operations (must match fusion_viewer for calibration compatibility):
     * 1. Apply calibration in SENSOR frame (hard-iron, soft-iron)
     * 2. Then apply axis remap to convert to BODY frame
     * 
     * This ensures calibration values from the viewer work directly.
     */
    
    /* Step 1: Apply magnetometer calibration in sensor frame */
    FusionVector cal_mag = FusionModelMagnetic(raw_mag, mag_soft_iron, mag_hard_iron);
    
    /* Step 2: Apply coordinate transform for LIS2MDL on back of PCB
     * Sensor → Device body frame: X = -sensor_X, Y = +sensor_Y, Z = -sensor_Z
     *
     * Device body frame represents the sensor flat on a table, LED facing North:
     *   X = East (right edge)
     *   Y = North (LED/light side)
     *   Z = Up (top face toward sky)
     *
     * At identity quaternion (1,0,0,0) the device is flat with LED pointing North.
     * This matches the FlySight2-Coordinate-Systems.md convention and keeps the
     * firmware output intuitive for bench testing. Downstream consumers (BASElineXR,
     * sensor fusion viewer) handle any additional frame transforms needed for
     * mounted/flight configurations.
     *
     * Note: A CG-mounted sensor could instead map directly to an aerodynamic NED
     * body frame aligned with the vehicle chord line, but the flat-on-table
     * convention is simpler and all existing calibrations assume it.
     */
    last_mag = (FusionVector){
        .axis = {
            .x = -cal_mag.axis.x,
            .y = +cal_mag.axis.y,
            .z = -cal_mag.axis.z,
        }
    };
    mag_valid = true;
}

void FS_Fusion_UpdateIMU(uint32_t time_ms,
                         int32_t wx, int32_t wy, int32_t wz,
                         int32_t ax, int32_t ay, int32_t az)
{
    if (!fusion_enabled || !fusion_active) return;
    
    /* Calculate delta time */
    float dt;
    if (last_imu_time == 0) {
        /* First sample - use nominal rate */
        dt = 1.0f / (float)SAMPLE_RATE_HZ;
    } else {
        /* Calculate actual dt from timestamps */
        uint32_t dt_ms = time_ms - last_imu_time;
        if (dt_ms == 0) dt_ms = 1;      /* Prevent divide by zero */
        if (dt_ms > 100) dt_ms = 100;   /* Cap at 100ms to prevent huge jumps */
        dt = (float)dt_ms / 1000.0f;
    }
    last_imu_time = time_ms;
    
    /* Convert gyro from integer (deg/s*1000) to float (deg/s) */
    FusionVector gyroscope = {
        .axis = {
            .x = (float)wx * GYRO_SCALE,
            .y = (float)wy * GYRO_SCALE,
            .z = (float)wz * GYRO_SCALE,
        }
    };
    
    /* Apply static gyro bias from config */
    gyroscope.axis.x -= gyro_bias.axis.x;
    gyroscope.axis.y -= gyro_bias.axis.y;
    gyroscope.axis.z -= gyro_bias.axis.z;
    
    /* Also apply runtime bias estimation (FusionBias refines on top of static) */
    gyroscope = FusionBiasUpdate(&bias, gyroscope);
    
    /* Convert accel from integer (g*100000) to float (g) */
    FusionVector raw_accel = {
        .axis = {
            .x = (float)ax * ACCEL_SCALE,
            .y = (float)ay * ACCEL_SCALE,
            .z = (float)az * ACCEL_SCALE,
        }
    };
    
    /* Apply accelerometer scale matrix calibration: corrected = M * raw */
    FusionVector accelerometer = FusionMatrixMultiply(accel_matrix, raw_accel);
    
    /* IMU data stays in device body frame (flat on table, LED = North):
     *   X = East, Y = North (LED), Z = Up
     *
     * The x-io Fusion library receives data in this frame and outputs
     * quaternions accordingly. Downstream consumers handle any additional
     * transforms for mounted/flight configurations.
     * See mag update comment for full convention rationale.
     */
    
    /* Run AHRS update */
    if (mag_valid) {
        /* 9-DOF update with magnetometer */
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, last_mag, dt);
    } else {
        /* 6-DOF update without magnetometer */
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    }
    
    /* Get quaternion output */
    FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    
    /* Get Euler angles */
    FusionEuler euler = FusionQuaternionToEuler(q);
    
    /* Update output structure */
    fusion_data.time = time_ms;
    fusion_data.q_w = q.element.w;
    fusion_data.q_x = q.element.x;
    fusion_data.q_y = q.element.y;
    fusion_data.q_z = q.element.z;
    fusion_data.roll = euler.angle.roll;
    fusion_data.pitch = euler.angle.pitch;
    fusion_data.heading = euler.angle.yaw;
    
    /* Normalize heading to 0-360 */
    if (fusion_data.heading < 0.0f) {
        fusion_data.heading += 360.0f;
    }
    
    /* Get flags */
    FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs);
    fusion_data.initialising = flags.initialising;
    
    /* Update status */
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrs);
    fusion_status.accelerationError = states.accelerationError;
    fusion_status.magneticError = states.magneticError;
    fusion_status.accelerometerIgnored = states.accelerometerIgnored;
    fusion_status.magnetometerIgnored = states.magnetometerIgnored;
    fusion_status.angularRateRecovery = flags.angularRateRecovery;

    /* Quaternion is logged as part of the IMU record via active_control.c */
}

const FS_Fusion_Data_t* FS_Fusion_GetData(void)
{
    return &fusion_data;
}

void FS_Fusion_GetQuaternion(float q[4])
{
    if (q == NULL) return;
    
    q[0] = fusion_data.q_w;
    q[1] = fusion_data.q_x;
    q[2] = fusion_data.q_y;
    q[3] = fusion_data.q_z;
}

const FS_Fusion_Status_t* FS_Fusion_GetStatus(void)
{
    return &fusion_status;
}

bool FS_Fusion_IsValid(void)
{
    return fusion_active && !fusion_data.initialising;
}

void FS_Fusion_SetMagHardIron(FusionVector offset)
{
    mag_hard_iron = offset;
}

void FS_Fusion_SetMagSoftIron(FusionMatrix matrix)
{
    mag_soft_iron = matrix;
}

FusionVector FS_Fusion_GetLinearAccel(void)
{
    return FusionAhrsGetLinearAcceleration(&ahrs);
}

FusionVector FS_Fusion_GetEarthAccel(void)
{
    return FusionAhrsGetEarthAcceleration(&ahrs);
}
