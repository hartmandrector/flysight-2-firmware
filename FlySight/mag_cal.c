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

#include "mag_cal.h"
#include "app_conf.h"
#include "ff.h"
#include "fusion_integration.h"
#include "log.h"
#include "motion_fx.h"
#include "main.h"
#include "stm32_seq.h"
#include <math.h>

/*
 * Persistent calibration cache file on the SD card.
 * Layout: float hi_gauss[3]  (12 bytes)
 *         uint8_t quality     ( 1 byte)
 * sizeof(MagCal_FileData_t) may be 16 due to trailing alignment padding;
 * we always write and read with sizeof(), so it is self-consistent.
 */
#define MAGCAL_BIN_PATH   "/MAGCAL.BIN"

typedef struct
{
    float   hi_gauss[3];   /* hard iron offset in gauss */
    uint8_t quality;       /* MagCal_Quality_t cast to uint8_t */
} MagCal_FileData_t;

/*
 * Unit conversions
 *
 * LIS2MDL produces gauss stored as int16_t * 1000 (milligauss).
 * MotionFX MagCal wants uT/50.
 *   1 gauss = 100 uT
 *   1 uT/50 = 50 uT ... wait, uT/50 means "the value represents uT divided by 50"
 *   so 1 unit of uT/50 = 1/50 uT ... no.
 *   The field is labelled [uT/50], meaning the physical quantity divided by 50.
 *   So: value_ut50 = physical_uT / 50
 *   1 gauss = 100 uT  =>  value_ut50 = 100 / 50 = 2
 *   Therefore: milligauss_int16 / 1000 [gauss] * 2 [uT/50 per gauss] = milligauss / 500
 *
 * hi_bias output is also in [uT/50]:
 *   hi_gauss = hi_bias_ut50 / 2
 */
#define MILLIGAUSS_TO_UT50   (1.0f / 500.0f)   /* int16 mg -> float uT/50 */
#define UT50_TO_GAUSS        (0.5f)             /* float uT/50 -> float gauss */

/*
 * MagCal is initialised with the nominal inter-call period (ms).
 * The library also uses the time_stamp field in each input for accurate
 * timing, so this value is used mainly for algorithm initialisation.
 *
 * Default Mag_ODR = 0 → 10 Hz → 100 ms period.
 */
#define MAGCAL_SAMPLE_PERIOD_MS   100   /* 10 Hz default mag ODR */

/* ---- Module-level persistent state (survives active/sleep transitions) ---- */

/* Best calibration quality seen since boot/hard-reset */
static MagCal_Quality_t s_quality = MAG_CAL_QUALITY_UNKNOWN;

/* Hard iron estimate in gauss, corresponding to s_quality */
static float s_hi_gauss[3] = {0.0f, 0.0f, 0.0f};

/* Set when a quality improvement needs to be persisted to disk */
static volatile bool s_save_pending = false;

/* Counts samples processed by the task */
static uint32_t s_update_count = 0;

/* Set every LOG_INTERVAL processed samples to write a status line to EVENT.CSV */
static bool s_log_pending = false;

/* Most recent raw output from MotionFX_MagCal_getParams */
static float s_last_hi_bias[3] = {0.0f, 0.0f, 0.0f};
static MagCal_Quality_t s_last_quality = MAG_CAL_QUALITY_UNKNOWN;
static MFX_MagCal_quality_t s_last_raw_quality = MFX_MAGCALUNKNOWN;

/* Last input values (uT/50) saved for diagnostic logging */
static float s_last_data_in[3] = {0.0f, 0.0f, 0.0f};

/* Pending input buffered from ISR for processing in task context */
static volatile bool s_run_pending = false;
static volatile int16_t s_pend_x, s_pend_y, s_pend_z;
static volatile int      s_pend_ts;

#define MAGCAL_LOG_INTERVAL   50   /* ~5 s at 10 Hz; reduce after diagnosis */

/* Set once on first task execution to log the runtime state address and gate byte */
static bool s_first_task_run = false;

/* ---- Internal helpers ---- */

static void MagCal_Save(void);  /* forward declaration */
static MagCal_Quality_t MFXQualityMap(MFX_MagCal_quality_t q);  /* forward declaration */
static void ApplyToFusion(void);  /* forward declaration */

static const char * const quality_names[] = { "UNKNOWN", "POOR", "OK", "GOOD" };

static void MagCal_SaveTask(void)
{
    /* One-time diagnostic: log state_base and gate byte to confirm whether
     * the constant-pool gate-byte hack actually worked at runtime. */
    if (!s_first_task_run)
    {
        const uint32_t fn = (uint32_t)(MotionFX_MagCal_init) & ~1U;
        const uint32_t sb = *(const uint32_t *)(fn + 0x10C);
        const uint8_t  gb = *((volatile uint8_t *)(sb + 964));
        const uint8_t  sf = *((volatile uint8_t *)(sb + 0x48));
        FS_Log_WriteEvent("MagCal diag: state=0x%lx gate=%d sf=%d", (unsigned long)sb, (int)gb, (int)sf);
        s_first_task_run = true;
    }

    if (s_run_pending)
    {
        MFX_MagCal_input_t data_in;
        /* Zero-init so the upper 3 bytes of cal_quality (a 4-byte int in our
         * build but written as 1 byte by the library's strb) stay zero.
         * Without this, uninitialized stack bytes produce rq garbage. */
        MFX_MagCal_output_t data_out = {0};
        MagCal_Quality_t new_quality;

        s_run_pending = false;

        /* Convert milligauss to uT/50 */
        data_in.mag[0]    = (float)s_pend_x * MILLIGAUSS_TO_UT50;
        data_in.mag[1]    = (float)s_pend_y * MILLIGAUSS_TO_UT50;
        data_in.mag[2]    = (float)s_pend_z * MILLIGAUSS_TO_UT50;
        data_in.time_stamp = (int)s_pend_ts;

        /* Run calibration step — safe here in main-loop task context */
        s_last_data_in[0] = data_in.mag[0];
        s_last_data_in[1] = data_in.mag[1];
        s_last_data_in[2] = data_in.mag[2];
        MotionFX_MagCal_run(&data_in);
        MotionFX_MagCal_getParams(&data_out);

        s_last_raw_quality = data_out.cal_quality;
        new_quality = MFXQualityMap(data_out.cal_quality);

        s_last_quality    = new_quality;
        s_last_hi_bias[0] = data_out.hi_bias[0];
        s_last_hi_bias[1] = data_out.hi_bias[1];
        s_last_hi_bias[2] = data_out.hi_bias[2];

        s_update_count++;
        if ((s_update_count % MAGCAL_LOG_INTERVAL) == 0)
        {
            s_log_pending = true;
        }

        if (new_quality > s_quality)
        {
            s_quality = new_quality;
            /* Guard against NaN/Inf — treat as 0 rather than corrupting MAGCAL.BIN */
            s_hi_gauss[0] = isfinite(data_out.hi_bias[0]) ? data_out.hi_bias[0] * UT50_TO_GAUSS : 0.0f;
            s_hi_gauss[1] = isfinite(data_out.hi_bias[1]) ? data_out.hi_bias[1] * UT50_TO_GAUSS : 0.0f;
            s_hi_gauss[2] = isfinite(data_out.hi_bias[2]) ? data_out.hi_bias[2] * UT50_TO_GAUSS : 0.0f;
            ApplyToFusion();
            s_save_pending = true;
        }
    }

    if (s_log_pending)
    {
        /* Use 99999 sentinel to flag NaN/Inf in hi_bias (ARM: (long)NaN == 0) */
        long hx = isfinite(s_last_hi_bias[0]) ? (long)(s_last_hi_bias[0] * 1000.0f) : 99999L;
        long hy = isfinite(s_last_hi_bias[1]) ? (long)(s_last_hi_bias[1] * 1000.0f) : 99999L;
        long hz = isfinite(s_last_hi_bias[2]) ? (long)(s_last_hi_bias[2] * 1000.0f) : 99999L;
        s_log_pending = false;
        FS_Log_WriteEvent("MagCal n=%lu rq=%d hi=[%ld,%ld,%ld] in=[%ld,%ld,%ld]",
            (unsigned long)s_update_count,
            (int)s_last_raw_quality,
            hx, hy, hz,
            (long)(s_last_data_in[0] * 1000.0f),
            (long)(s_last_data_in[1] * 1000.0f),
            (long)(s_last_data_in[2] * 1000.0f));
    }
    if (s_save_pending)
    {
        s_save_pending = false;
        FS_Log_WriteEvent("MagCal save q=%s hi=[%ld,%ld,%ld]e-3 gauss",
            quality_names[s_quality],
            (long)(s_hi_gauss[0] * 1000.0f),
            (long)(s_hi_gauss[1] * 1000.0f),
            (long)(s_hi_gauss[2] * 1000.0f));
        MagCal_Save();
    }
}

static MagCal_Quality_t MFXQualityMap(MFX_MagCal_quality_t q)
{
    switch (q)
    {
    case MFX_MAGCALPOOR:   return MAG_CAL_QUALITY_POOR;
    case MFX_MAGCALOK:     return MAG_CAL_QUALITY_OK;
    case MFX_MAGCALGOOD:   return MAG_CAL_QUALITY_GOOD;
    default:               return MAG_CAL_QUALITY_UNKNOWN;
    }
}

static void ApplyToFusion(void)
{
    FusionVector hi;
    hi.axis.x = s_hi_gauss[0];
    hi.axis.y = s_hi_gauss[1];
    hi.axis.z = s_hi_gauss[2];
    FS_Fusion_SetMagHardIron(hi);
}

/*
 * Load calibration from MAGCAL.BIN on the SD card.
 * Called only once on cold boot (s_quality == UNKNOWN).
 * FATFS is already mounted by active_mode.c before MagCal_Init() is reached.
 */
static void MagCal_Load(void)
{
    FIL f;
    UINT br;
    MagCal_FileData_t data;

    /* Skip if we already have an in-RAM calibration (sleep/wake cycle) */
    if (s_quality != MAG_CAL_QUALITY_UNKNOWN)
    {
        return;
    }

    if (f_open(&f, MAGCAL_BIN_PATH, FA_READ) != FR_OK)
    {
        return;
    }

    if ((f_read(&f, &data, sizeof(data), &br) == FR_OK) &&
        (br == sizeof(data)) &&
        (data.quality > (uint8_t)MAG_CAL_QUALITY_UNKNOWN) &&
        (data.quality <= (uint8_t)MAG_CAL_QUALITY_GOOD))
    {
        s_quality     = (MagCal_Quality_t)data.quality;
        s_hi_gauss[0] = data.hi_gauss[0];
        s_hi_gauss[1] = data.hi_gauss[1];
        s_hi_gauss[2] = data.hi_gauss[2];
    }

    f_close(&f);
}

/*
 * Save current calibration to MAGCAL.BIN.
 * Called whenever quality improves so the best-seen calibration is always
 * on disk.  FATFS is held by active_mode.c for the duration of active mode.
 */
static void MagCal_Save(void)
{
    FIL f;
    UINT bw;
    MagCal_FileData_t data;

    data.hi_gauss[0] = s_hi_gauss[0];
    data.hi_gauss[1] = s_hi_gauss[1];
    data.hi_gauss[2] = s_hi_gauss[2];
    data.quality     = (uint8_t)s_quality;

    if (f_open(&f, MAGCAL_BIN_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        return;
    }

    f_write(&f, &data, sizeof(data), &bw);
    f_close(&f);
}

/* ---- Public API ---- */

void MagCal_Init(void)
{
    char libver[35];

    /* Restore calibration from SD card on cold boot */
    MagCal_Load();

    /* Register deferred-save task (safe to call multiple times) */
    UTIL_SEQ_RegTask(1<<CFG_TASK_FS_MAGCAL_SAVE_ID, UTIL_SEQ_RFU, MagCal_SaveTask);

    /*
     * All three MagCal functions guard on a gate byte at library_state+964.
     * MotionFX_initialize() is the only function that sets it, but on
     * STM32WB5MMGHX that function hangs: the device CPUID bits[11:0]=0x241
     * matches none of the expected values (0x450,0x483,0x485,0x500), so the
     * code falls into a hardware-semaphore spinloop that never resolves.
     *
     * Workaround: read the library global state address from the constant
     * pool of MotionFX_MagCal_init (verified offset 0x10C by objdump of the
     * .a file), then write the single gate byte directly.  This is safe:
     * reads a 4-byte-aligned word from linked Flash, writes one byte to BSS.
     */
    {
        /* Strip Thumb bit to get raw instruction address */
        const uint32_t fn = (uint32_t)(MotionFX_MagCal_init) & ~1U;
        /* Pool entry at fn+0x10C holds the library global state pointer */
        const uint32_t state_base = *(const uint32_t *)(fn + 0x10C);
        /* Gate byte: enables all three MagCal functions */
        *((volatile uint8_t *)(state_base + 964)) = 1;   /* state + 0x3C4 */

        /* (Re)start the MagCal algorithm */
        MotionFX_MagCal_init(MAGCAL_SAMPLE_PERIOD_MS, 1);

        /*
         * Sphere-fitter enable flag: MagCal_init(100,1) clears state[0x48]
         * to 0 (r6=0 at lib offset 0xb0), but MagCal_run checks state[0x48]
         * != 0 at offset 0x3fa before invoking the sphere fitter.  Without
         * this, MagCal_run stores samples but never computes a calibration.
         * MotionFX_initialize() normally sets this flag via internal sub-init
         * routines that we bypass.  Set it explicitly here, after the reset.
         */
        *((volatile uint8_t *)(state_base + 0x48)) = 1;
    }

    MotionFX_GetLibVersion(libver);

    /*
     * If a valid calibration already exists (from RAM or just loaded from
     * disk), apply it to fusion immediately so hard iron correction is not
     * lost on mode re-entry (FS_Fusion_Init reloads from config, overwriting
     * any runtime value set last session).
     */
    if (s_quality >= MAG_CAL_QUALITY_OK)
    {
        ApplyToFusion();
    }

    FS_Log_WriteEvent("MagCal init q=%s lib=%s", quality_names[s_quality], libver);
}

void MagCal_Update(int16_t x, int16_t y, int16_t z)
{
    /* Buffer latest sample for processing in task context.
     * MotionFX_MagCal_run must NOT be called from ISR — it uses
     * significant stack and FPU state, which corrupts the main fusion engine. */
    s_pend_x  = x;
    s_pend_y  = y;
    s_pend_z  = z;
    s_pend_ts = (int)HAL_GetTick();
    s_run_pending = true;
    UTIL_SEQ_SetTask(1<<CFG_TASK_FS_MAGCAL_SAVE_ID, CFG_SCH_PRIO_1);
}

MagCal_Quality_t MagCal_GetQuality(void)
{
    return s_quality;
}

bool MagCal_GetHardIron(float hi_gauss[3])
{
    if (s_quality == MAG_CAL_QUALITY_UNKNOWN)
    {
        return false;
    }

    hi_gauss[0] = s_hi_gauss[0];
    hi_gauss[1] = s_hi_gauss[1];
    hi_gauss[2] = s_hi_gauss[2];

    return true;
}
