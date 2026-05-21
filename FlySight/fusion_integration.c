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
 * Uses the ST MotionFX library (9-axis Kalman-based AHRS) with:
 *   - Gyroscope bias learning (static mode)
 *   - Hard-iron calibration via mag_cal.c
 *   - Coordinate frame pre-remapping for LIS2MDL magnetometer
 *
 * MotionFX_initialize() hangs on this platform (it waits on an STM32H7 HSEM
 * address).  We bypass it by setting the library's internal gate byte via a
 * constant-pool pointer read, then manually writing the values that the
 * success path of MotionFX_initialize() would have written.
 *
 * Body frame convention (identity quaternion = device flat, LED pointing North):
 *   X = East, Y = North, Z = Up  (ENU)
 *
 * LIS2MDL is on the BACK of the PCB.  Sensor axes are remapped to body frame
 * before being passed to MotionFX:
 *   body_X = -sensor_X,  body_Y = +sensor_Y,  body_Z = -sensor_Z
 *
 * Data flow:
 *   MAG (~100 Hz) --> FS_Fusion_UpdateMag() --> stores calibrated body-frame
 *                                               mag vector (uT/50)
 *   IMU (~100 Hz) --> FS_Fusion_UpdateIMU() --> MotionFX_propagate + _update,
 *                                               quaternion/euler extracted
 */

#include "fusion_integration.h"
#include "motion_fx.h"
#include "config.h"
#include <string.h>

/* -------------------------------------------------------------------------
 * Unit conversion
 * ------------------------------------------------------------------------- */
#define GYRO_SCALE    (1.0f / 1000.0f)    /* dps*1000  -> dps  */
#define ACCEL_SCALE   (1.0f / 100000.0f)  /* g*100000  -> g    */
#define MG_TO_UT50    (1.0f / 500.0f)     /* milligauss -> uT/50 */
#define GAUSS_TO_UT50 (2.0f)              /* gauss -> uT/50    */

/* Number of output samples to discard while the filter converges */
#define SAMPLES_TO_DISCARD  15

/* Set to 1 to run 6-DOF (accel + gyro only, no magnetometer) for diagnostic
 * purposes.  Change back to 0 to restore full 9-DOF fusion. */
#define MFX_6DOF_ONLY  0

/* MotionFX user-state size (confirmed by disassembly: MotionFX_GetStateSize) */
#define MFX_STATE_SIZE  2432

/* -------------------------------------------------------------------------
 * Module state
 * ------------------------------------------------------------------------- */

/* MotionFX 2432-byte user state + last output */
static uint8_t s_mfxstate[MFX_STATE_SIZE];
static MFX_output_t s_mfx_out;

/* Hard iron calibration, stored in uT/50 in sensor frame.
 * Set by FS_Fusion_SetMagHardIron() which receives gauss from callers. */
static float s_hi_ut50[3];

/* Last calibrated magnetometer vector in body frame, uT/50.
 * Written by FS_Fusion_UpdateMag(), consumed by FS_Fusion_UpdateIMU(). */
static float s_mag_ut50[3];
static bool  s_mag_fresh;

static uint32_t s_last_imu_time;
static int      s_discard_count;

static bool fusion_enabled;
static bool fusion_active;

static FS_Fusion_Data_t   fusion_data;
static FS_Fusion_Status_t fusion_status;

/* -------------------------------------------------------------------------
 * MotionFX_initialize() bypass
 *
 * Every public MotionFX function checks a "gate byte" at offset 0x3C4 within
 * a library-internal global state blob and returns immediately (no-op) when
 * it is zero.  MotionFX_initialize() sets that byte in its success path, but
 * hangs before reaching it on this hardware.
 *
 * We locate the global state via the constant pool embedded in
 * MotionFX_initialize itself (word at fn+0x154 is the pointer), then force
 * the gate byte to 1 before touching any other API.
 *
 * The 2432-byte per-instance user state (s_mfxstate) is separate.  We
 * initialise it by writing the same values that MotionFX_initialize's success
 * path would have written; MotionFX_setKnobs then overwrites the fields we
 * actually care about.
 * ------------------------------------------------------------------------- */
static void MFX_SetGateByte(void)
{
    const uint32_t fn = (uint32_t)(MotionFX_initialize) & ~1U;
    const uint32_t gs = *(const uint32_t *)(fn + 0x154U);
    *((volatile uint8_t *)(gs + 0x3C4U)) = 1U;
}

/*
 * Decode Thumb-2 BL at MotionFX_initialize+0xb8 to find MFX_emptyAttitude,
 * the internal Kalman covariance initializer, then call it on our state buffer.
 *
 * MotionFX_initialize's success path calls MFX_emptyAttitude first.  That
 * function uses DataHist_parameters to compute proper initial Kalman P/Q/R
 * covariance matrices and writes them — along with several non-knob scalar
 * fields at state offsets 8, 12, 16, 20, 36, 48, 88–184 — into the state
 * buffer.  None of these were replicated by the previous simple field-write
 * approach, leaving the filter with degenerate all-zero covariances (K≈0),
 * which caused the filter to behave as a pure gyro integrator with no
 * accelerometer tilt correction.
 *
 * MFX_emptyAttitude is a local (non-exported) symbol, so it cannot be
 * referenced by name.  We locate it at run time by decoding the BL T1
 * instruction at MotionFX_initialize+0xb8 (the first internal call in the
 * success path, before the orientation-string strcpy calls).
 */
static void MFX_CallEmptyAttitude(void *state)
{
    /* MotionFX functions are Thumb; clear bit 0 to get the code address */
    const uint16_t *bl = (const uint16_t *)
        (((uintptr_t)(const void *)MotionFX_initialize & ~1U) + 0xb8U);

    /* Decode BL T1: two consecutive Thumb-2 halfwords                    */
    /* hw1: [15:11]=11110  [10]=S  [9:0]=imm10                            */
    /* hw2: [15:14]=11  [13]=J1  [12]=1  [11]=J2  [10:0]=imm11           */
    /* imm32 = SignExtend(S:I1:I2:imm10:imm11:0, 25)                      */
    /* I1 = NOT(J1 XOR S);  I2 = NOT(J2 XOR S)                           */
    uint16_t hw1   = bl[0];
    uint16_t hw2   = bl[1];
    uint32_t S     = (uint32_t)(hw1 >> 10) & 1U;
    uint32_t imm10 = (uint32_t)hw1         & 0x3FFU;
    uint32_t J1    = (uint32_t)(hw2 >> 13) & 1U;
    uint32_t J2    = (uint32_t)(hw2 >> 11) & 1U;
    uint32_t imm11 = (uint32_t)hw2         & 0x7FFU;
    uint32_t I1    = (~(J1 ^ S)) & 1U;
    uint32_t I2    = (~(J2 ^ S)) & 1U;
    int32_t  off   = (int32_t)((S << 24) | (I1 << 23) | (I2 << 22) |
                               (imm10 << 12) | (imm11 << 1));
    if (S) off |= (int32_t)0xFF000000;    /* sign-extend from bit 24 */

    /* PC at BL = instruction address + 4; target = PC + signed offset */
    uintptr_t target = (uintptr_t)(bl + 2) + (uintptr_t)(intptr_t)off;

    typedef void (*init_fn_t)(void *);
    ((init_fn_t)(target | 1U))(state);    /* bit 0 = 1: Thumb interworking */
}

static void MFX_ManualInitState(void)
{
    memset(s_mfxstate, 0, MFX_STATE_SIZE);

    /* Call MFX_emptyAttitude (located dynamically via BL decode above) to
     * properly initialise the Kalman covariance matrices and internal scalar
     * fields.  MFX_ConfigureKnobs → MotionFX_setKnobs will then override
     * all user-visible knob fields (orientations, thresholds, ATime, etc.). */
    MFX_CallEmptyAttitude(s_mfxstate);
}

static void MFX_ConfigureKnobs(void)
{
    MFX_knobs_t knobs;
    MotionFX_getKnobs((MFXState_t)s_mfxstate, &knobs);

    memcpy(knobs.acc_orientation,  "enu", 4);
    memcpy(knobs.gyro_orientation, "enu", 4);
    memcpy(knobs.mag_orientation,  "wnd", 4); /* LIS2MDL driver output: W, N, U */
    knobs.output_type              = MFX_ENGINE_OUTPUT_ENU;
    knobs.LMode                    = 1;           /* static gyro bias learning */
    knobs.modx                     = 1;           /* no decimation             */
    knobs.gbias_acc_th_sc          = 2.0f * 0.000765f;
    knobs.gbias_gyro_th_sc         = 2.0f * 0.002f;
    knobs.gbias_mag_th_sc          = 2.0f * 0.0015f;

    MotionFX_setKnobs((MFXState_t)s_mfxstate, &knobs);
#if MFX_6DOF_ONLY
    MotionFX_enable_9X((MFXState_t)s_mfxstate, MFX_ENGINE_DISABLE);
    MotionFX_enable_6X((MFXState_t)s_mfxstate, MFX_ENGINE_ENABLE);
#else
    MotionFX_enable_6X((MFXState_t)s_mfxstate, MFX_ENGINE_DISABLE);
    MotionFX_enable_9X((MFXState_t)s_mfxstate, MFX_ENGINE_ENABLE);
#endif
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void FS_Fusion_Init(void)
{
    const FS_Config_Data_t *cfg = FS_Config_Get();

    fusion_enabled = (bool)cfg->enable_fusion;
    if (!fusion_enabled) return;

    /* 1. Open the library gate so all MotionFX functions execute normally */
    MFX_SetGateByte();

    /* 2. Manually initialise the per-instance user state buffer */
    MFX_ManualInitState();

    /* 3. Apply knobs and enable 9-axis fusion */
    MFX_ConfigureKnobs();

    /* 4. Clear runtime state */
    s_last_imu_time = 0;
    s_discard_count = 0;
    s_mag_fresh     = false;
    s_hi_ut50[0] = s_hi_ut50[1] = s_hi_ut50[2] = 0.0f;
    s_mag_ut50[0] = s_mag_ut50[1] = s_mag_ut50[2] = 0.0f;
    memset(&s_mfx_out, 0, sizeof(s_mfx_out));

    fusion_data.time        = 0;
    fusion_data.q_w         = 1.0f;
    fusion_data.q_x         = 0.0f;
    fusion_data.q_y         = 0.0f;
    fusion_data.q_z         = 0.0f;
    fusion_data.heading     = 0.0f;
    fusion_data.pitch       = 0.0f;
    fusion_data.roll        = 0.0f;
    fusion_data.initialising = true;
    memset(&fusion_status, 0, sizeof(fusion_status));
}

void FS_Fusion_Start(void)
{
    if (!fusion_enabled) return;

    /* Re-initialise state so each recording starts with a clean filter */
    MFX_ManualInitState();
    MFX_ConfigureKnobs();

    fusion_active   = true;
    s_last_imu_time = 0;
    s_discard_count = 0;
    s_mag_fresh     = false;

    fusion_data.initialising = true;
}

void FS_Fusion_Stop(void)
{
    fusion_active = false;
}

void FS_Fusion_UpdateMag(int16_t x, int16_t y, int16_t z)
{
    if (!fusion_enabled || !fusion_active) return;

    /* Convert milligauss → uT/50 (sensor frame) */
    float sx = (float)x * MG_TO_UT50;
    float sy = (float)y * MG_TO_UT50;
    float sz = (float)z * MG_TO_UT50;

    /* Subtract hard-iron calibration (sensor frame, uT/50) */
    sx -= s_hi_ut50[0];
    sy -= s_hi_ut50[1];
    sz -= s_hi_ut50[2];

    /* Pass driver-frame values directly.  The LIS2MDL is mounted on the bottom
     * of the PCB: raw sensor axes are W, N, D.  The mag.c driver negates Z
     * (Down → Up), so the driver output frame is W, N, U.  MotionFX is told
     * mag_orientation="wnu" and applies the West→East negation internally;
     * no manual axis flip is needed here. */
    s_mag_ut50[0] = sx;
    s_mag_ut50[1] = sy;
    s_mag_ut50[2] = sz;

    s_mag_fresh = true;
}

void FS_Fusion_UpdateIMU(uint32_t time_ms,
                         int32_t wx, int32_t wy, int32_t wz,
                         int32_t ax, int32_t ay, int32_t az)
{
    if (!fusion_enabled || !fusion_active) return;

    /* Compute actual delta-t from timestamps */
    float dt;
    if (s_last_imu_time == 0)
    {
        dt = 0.01f;   /* nominal 100 Hz for very first sample */
    }
    else
    {
        uint32_t dt_ms = time_ms - s_last_imu_time;
        if (dt_ms == 0)   dt_ms = 1;    /* guard against duplicate timestamps */
        if (dt_ms > 100)  dt_ms = 100;  /* cap at 100 ms to limit filter jumps */
        dt = (float)dt_ms * 0.001f;
    }
    s_last_imu_time = time_ms;

    /* Build MotionFX input (body frame, already correct for IMU) */
    MFX_input_t data_in;
    data_in.gyro[0] = (float)wx * GYRO_SCALE;
    data_in.gyro[1] = (float)wy * GYRO_SCALE;
    data_in.gyro[2] = (float)wz * GYRO_SCALE;
    data_in.acc[0]  = (float)ax * ACCEL_SCALE;
    data_in.acc[1]  = (float)ay * ACCEL_SCALE;
    data_in.acc[2]  = (float)az * ACCEL_SCALE;
    data_in.mag[0]  = s_mag_ut50[0];
    data_in.mag[1]  = s_mag_ut50[1];
    data_in.mag[2]  = s_mag_ut50[2];

    /* Predict step: gyro integration */
    MotionFX_propagate((MFXState_t)s_mfxstate, &s_mfx_out, &data_in, &dt);

    /* Correction step:
     * - 6-DOF: call every IMU sample so accel tilt correction runs at full rate.
     * - 9-DOF: call only when a fresh mag reading is available. */
#if MFX_6DOF_ONLY
    MotionFX_update((MFXState_t)s_mfxstate, &s_mfx_out, &data_in, &dt, NULL);
    s_mag_fresh = false;
#else
    if (s_mag_fresh)
    {
        MotionFX_update((MFXState_t)s_mfxstate, &s_mfx_out, &data_in, &dt, NULL);
        s_mag_fresh = false;
    }
#endif

    /* Discard first N outputs while the filter converges */
    if (s_discard_count < SAMPLES_TO_DISCARD)
    {
        s_discard_count++;
        fusion_data.initialising = true;
        return;
    }

    /* MotionFX quaternion order: [x, y, z, w] (index 3 = scalar w) */
    fusion_data.time = time_ms;
    fusion_data.q_w  = s_mfx_out.quaternion[3];
    fusion_data.q_x  = s_mfx_out.quaternion[0];
    fusion_data.q_y  = s_mfx_out.quaternion[1];
    fusion_data.q_z  = s_mfx_out.quaternion[2];

    /* Euler angles: rotation[0]=yaw, [1]=pitch, [2]=roll */
    float heading = s_mfx_out.rotation[0];
    if (heading < 0.0f) heading += 360.0f;
    fusion_data.heading     = heading;
    fusion_data.pitch       = s_mfx_out.rotation[1];
    fusion_data.roll        = s_mfx_out.rotation[2];
    fusion_data.initialising = false;

    /* MotionFX does not expose per-sample rejection flags; zero status fields */
    fusion_status.accelerationError    = 0.0f;
    fusion_status.magneticError        = 0.0f;
    fusion_status.accelerometerIgnored = false;
    fusion_status.magnetometerIgnored  = false;
    fusion_status.angularRateRecovery  = false;
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
    /* Callers (mag_cal.c, sensor_data.c) pass values in gauss.
     * Convert to uT/50 for use in FS_Fusion_UpdateMag(). */
    s_hi_ut50[0] = offset.axis.x * GAUSS_TO_UT50;
    s_hi_ut50[1] = offset.axis.y * GAUSS_TO_UT50;
    s_hi_ut50[2] = offset.axis.z * GAUSS_TO_UT50;
}

void FS_Fusion_SetMagSoftIron(FusionMatrix matrix)
{
    /* MotionFX does not support soft-iron correction; no-op. */
    (void)matrix;
}

FusionVector FS_Fusion_GetLinearAccel(void)
{
    /* MotionFX provides linear_acceleration in the device body frame. */
    return (FusionVector){{ s_mfx_out.linear_acceleration[0],
                            s_mfx_out.linear_acceleration[1],
                            s_mfx_out.linear_acceleration[2] }};
}

FusionVector FS_Fusion_GetEarthAccel(void)
{
    /* Earth-frame acceleration not directly available from MotionFX output. */
    return FUSION_VECTOR_ZERO;
}
