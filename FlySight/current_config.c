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

#include <string.h>

#include "current_config.h"
#include "baro_ble.h"
#include "hum_ble.h"
#include "accel_ble.h"
#include "gyro_ble.h"
#include "mag_ble.h"
#include "sensor_odr.h"

/* ── BLE packet sizes (bytes) ──────────────────────────────────────────── */
/* Source: Docs/sensorbleplanning_v2.md                                     */

#define CC_GPS_PACKET_SIZE    44u
#define CC_BARO_PACKET_SIZE   11u
#define CC_HUM_PACKET_SIZE     9u
#define CC_ACCEL_PACKET_SIZE  19u
#define CC_GYRO_PACKET_SIZE   27u
#define CC_MAG_PACKET_SIZE    13u

/* Conservative estimate for one ActiveLook pageClearAndDisplay packet:
 * 4-byte framing header + page ID + heading string + up to 4 data strings
 * + footer.  See Open Question #6 in CURRENT_CONFIG_DESIGN.md. */
#define CC_AL_PACKET_SIZE     60u

/* ActiveLook bandwidth warning threshold (percent of total budget) */
#define CC_AL_HIGH_BW_PCT     30u

/* ── Module state ───────────────────────────────────────────────────────── */

static CC_RuntimeConfig_t cc_runtime;

/* ── Internal helpers ───────────────────────────────────────────────────── */

static void cc_set_field(CC_U16Field_t *f, uint16_t requested, uint16_t effective,
                          CC_Source_t source, CC_PersistPolicy_t persist)
{
    f->requested = requested;
    f->effective = effective;
    f->source    = source;
    f->persist   = persist;
}

/* Increment revision counter on every mutation. */
static void cc_notify(void)
{
    cc_runtime.revision++;
}

static void CC_ApplyBleDividers(void); /* forward declaration */

/* ── BLE budget recomputation ───────────────────────────────────────────── */
/*
 * Implements the priority-based algorithm from CURRENT_CONFIG_DESIGN.md:
 *
 *  1. GPS bandwidth
 *  2. ActiveLook bandwidth
 *  3. Manual sensor bandwidth (non-zero requested divider)
 *  4. Remaining budget = LIMIT − GPS − AL − MANUAL
 *  5. Auto sensor full-ODR bandwidth
 *  6. If auto fits → effective divider = 1
 *  7. Otherwise → scale all auto dividers proportionally
 *  8. Clamp each divider to 0.5 Hz floor
 *  9. Update effective divider fields (source = CC_SRC_AUTO_CALC)
 * 10. Write summary stats and warning flags
 */
void CC_RecomputeBudget(void)
{
    /* ── Sensor descriptor table ──────────────────────────────────────── */
    typedef struct {
        bool            enabled;
        uint16_t        req_div;          /* copy of requested divider */
        CC_U16Field_t  *div_field;        /* points into cc_runtime */
        uint8_t         odr_idx;          /* ODR index (copy) */
        const uint16_t *odr_table;
        uint8_t         odr_table_len;
        uint16_t        pkt_size;
        uint16_t        hw_hz;            /* filled during init pass */
        bool            is_manual;        /* req_div != 0 */
    } sensor_t;

    sensor_t s[5] = {
        { cc_runtime.enable_baro,
          cc_runtime.ble_baro_divider.requested,  &cc_runtime.ble_baro_divider,
          (uint8_t)cc_runtime.baro_odr.effective,
          baro_odr_table,  BARO_ODR_TABLE_SIZE,  CC_BARO_PACKET_SIZE,  0, false },

        { cc_runtime.enable_hum,
          cc_runtime.ble_hum_divider.requested,   &cc_runtime.ble_hum_divider,
          (uint8_t)cc_runtime.hum_odr.effective,
          hum_odr_table,   HUM_ODR_TABLE_SIZE,   CC_HUM_PACKET_SIZE,   0, false },

        { cc_runtime.enable_imu,
          cc_runtime.ble_accel_divider.requested, &cc_runtime.ble_accel_divider,
          (uint8_t)cc_runtime.accel_odr.effective,
          accel_odr_table, ACCEL_ODR_TABLE_SIZE, CC_ACCEL_PACKET_SIZE, 0, false },

        { cc_runtime.enable_imu,
          cc_runtime.ble_gyro_divider.requested,  &cc_runtime.ble_gyro_divider,
          (uint8_t)cc_runtime.gyro_odr.effective,
          gyro_odr_table,  GYRO_ODR_TABLE_SIZE,  CC_GYRO_PACKET_SIZE,  0, false },

        { cc_runtime.enable_mag,
          cc_runtime.ble_mag_divider.requested,   &cc_runtime.ble_mag_divider,
          (uint8_t)cc_runtime.mag_odr.effective,
          mag_odr_table,   MAG_ODR_TABLE_SIZE,   CC_MAG_PACKET_SIZE,   0, false },
    };

    /* ── Pass 1: resolve HW rates ─────────────────────────────────────── */
    for (int i = 0; i < 5; i++)
    {
        if (!s[i].enabled) continue;
        s[i].hw_hz     = ODR_TO_HZ(s[i].odr_table, s[i].odr_table_len, s[i].odr_idx);
        s[i].is_manual = (s[i].req_div != 0);
    }

    /* ── Step 1: GPS bandwidth ────────────────────────────────────────── */
    uint32_t gps_bw = 0;
    if (cc_runtime.enable_gnss && cc_runtime.gnss_rate_ms.effective > 0)
        gps_bw = (1000u / cc_runtime.gnss_rate_ms.effective) * CC_GPS_PACKET_SIZE;

    /* ── Step 2: ActiveLook bandwidth ────────────────────────────────── */
    uint32_t al_bw = 0;
    if (cc_runtime.al_enabled && cc_runtime.al_rate_ms.effective > 0)
        al_bw = (1000u / cc_runtime.al_rate_ms.effective) * CC_AL_PACKET_SIZE;

    /* ── Step 3: Manual sensor bandwidth ─────────────────────────────── */
    uint32_t manual_bw = 0;
    for (int i = 0; i < 5; i++)
    {
        if (!s[i].enabled || !s[i].is_manual || s[i].hw_hz == 0) continue;
        manual_bw += (uint32_t)(s[i].hw_hz / s[i].req_div) * s[i].pkt_size;
    }

    /* ── Step 4: Remaining budget ─────────────────────────────────────── */
    int32_t remaining = (int32_t)BLE_SAFE_THROUGHPUT_LIMIT
                      - (int32_t)(gps_bw + al_bw + manual_bw);
    if (remaining < 0) remaining = 0;

    /* ── Step 5: Auto sensor full-ODR bandwidth ───────────────────────── */
    uint32_t auto_full_bw = 0;
    for (int i = 0; i < 5; i++)
    {
        if (!s[i].enabled || s[i].is_manual || s[i].hw_hz == 0) continue;
        auto_full_bw += (uint32_t)s[i].hw_hz * s[i].pkt_size;
    }

    /* ── Steps 6-8: Assign effective dividers ─────────────────────────── */
    cc_runtime.warning_flags &= ~CC_WARN_DIVIDER_AT_MIN_RATE;

    for (int i = 0; i < 5; i++)
    {
        if (!s[i].enabled || s[i].hw_hz == 0) continue;

        if (s[i].is_manual)
        {
            /* Manual: effective = requested; source already set at mutation time */
            s[i].div_field->effective = s[i].req_div;
        }
        else
        {
            /* Auto: compute scaled divider */
            uint16_t div = 1;

            if (remaining == 0)
            {
                /* GPS + AL + manual already blew the budget — floor auto sensors */
                div = (uint16_t)((uint32_t)s[i].hw_hz * 2u); /* 0.5 Hz minimum */
            }
            else if (auto_full_bw > (uint32_t)remaining)
            {
                float scale = (float)auto_full_bw / (float)remaining;
                div = (uint16_t)(scale + 0.5f);
                if (div < 1u) div = 1u;
            }
            /* else: auto sensors fit at full ODR — div stays 1 */

            /* Clamp to 0.5 Hz minimum floor */
            uint16_t max_div = (uint16_t)((uint32_t)s[i].hw_hz * 2u);
            if (max_div < 1u) max_div = 1u;
            if (div > max_div)
            {
                div = max_div;
                cc_runtime.warning_flags |= CC_WARN_DIVIDER_AT_MIN_RATE;
            }

            s[i].div_field->effective = div;
            s[i].div_field->source    = CC_SRC_AUTO_CALC;
        }
    }

    /* ── Step 9-10: Summary stats and warnings ───────────────────────── */
    uint32_t sensor_bw = 0;
    for (int i = 0; i < 5; i++)
    {
        if (!s[i].enabled || s[i].hw_hz == 0) continue;
        uint16_t eff = s[i].div_field->effective;
        if (eff == 0) eff = 1;
        sensor_bw += (uint32_t)(s[i].hw_hz / eff) * s[i].pkt_size;
    }

    cc_runtime.ble_sensor_bytes_per_sec       = sensor_bw;
    cc_runtime.ble_activelook_bytes_per_sec   = al_bw;
    cc_runtime.ble_estimated_bytes_per_sec    = gps_bw + al_bw + sensor_bw;
    cc_runtime.ble_budget_ok = (cc_runtime.ble_estimated_bytes_per_sec
                                <= BLE_SAFE_THROUGHPUT_LIMIT);

    if (!cc_runtime.ble_budget_ok)
        cc_runtime.warning_flags |= CC_WARN_BLE_OVER_BUDGET;
    else
        cc_runtime.warning_flags &= ~CC_WARN_BLE_OVER_BUDGET;

    if (al_bw > 0 &&
        ((al_bw * 100u) / BLE_SAFE_THROUGHPUT_LIMIT) > CC_AL_HIGH_BW_PCT)
        cc_runtime.warning_flags |= CC_WARN_AL_HIGH_BW;
    else
        cc_runtime.warning_flags &= ~CC_WARN_AL_HIGH_BW;

    CC_ApplyBleDividers();
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void CC_Init(const FS_Config_Data_t *file_config)
{
    memset(&cc_runtime, 0, sizeof(cc_runtime));

    /* GNSS rate */
    cc_set_field(&cc_runtime.gnss_rate_ms,
                 file_config->rate, file_config->rate,
                 CC_SRC_FILE, CC_PERSIST_NEVER);

    /* Sensor ODRs */
    cc_set_field(&cc_runtime.baro_odr,
                 file_config->baro_odr, file_config->baro_odr,
                 CC_SRC_FILE, CC_PERSIST_NEVER);
    cc_set_field(&cc_runtime.hum_odr,
                 file_config->hum_odr, file_config->hum_odr,
                 CC_SRC_FILE, CC_PERSIST_NEVER);
    cc_set_field(&cc_runtime.accel_odr,
                 file_config->accel_odr, file_config->accel_odr,
                 CC_SRC_FILE, CC_PERSIST_NEVER);
    cc_set_field(&cc_runtime.gyro_odr,
                 file_config->gyro_odr, file_config->gyro_odr,
                 CC_SRC_FILE, CC_PERSIST_NEVER);
    cc_set_field(&cc_runtime.mag_odr,
                 file_config->mag_odr, file_config->mag_odr,
                 CC_SRC_FILE, CC_PERSIST_NEVER);

    /* BLE dividers — 0 = auto-calculate */
    cc_set_field(&cc_runtime.ble_baro_divider,
                 file_config->ble_baro_divider, file_config->ble_baro_divider,
                 CC_SRC_FILE, CC_PERSIST_ON_REQUEST);
    cc_set_field(&cc_runtime.ble_hum_divider,
                 file_config->ble_hum_divider, file_config->ble_hum_divider,
                 CC_SRC_FILE, CC_PERSIST_ON_REQUEST);
    cc_set_field(&cc_runtime.ble_accel_divider,
                 file_config->ble_accel_divider, file_config->ble_accel_divider,
                 CC_SRC_FILE, CC_PERSIST_ON_REQUEST);
    cc_set_field(&cc_runtime.ble_gyro_divider,
                 file_config->ble_gyro_divider, file_config->ble_gyro_divider,
                 CC_SRC_FILE, CC_PERSIST_ON_REQUEST);
    cc_set_field(&cc_runtime.ble_mag_divider,
                 file_config->ble_mag_divider, file_config->ble_mag_divider,
                 CC_SRC_FILE, CC_PERSIST_ON_REQUEST);

    /* ActiveLook — al_rate is uint32_t in file config; clamp to uint16_t */
    uint16_t al_rate = (file_config->al_rate > 0xFFFFu)
                     ? 0xFFFFu
                     : (uint16_t)file_config->al_rate;
    cc_set_field(&cc_runtime.al_rate_ms,
                 al_rate, al_rate,
                 CC_SRC_FILE, CC_PERSIST_NEVER);
    cc_runtime.al_enabled = (file_config->al_mode != 0);

    /* Enable flags */
    cc_runtime.enable_gnss = (file_config->enable_gnss != 0);
    cc_runtime.enable_baro = (file_config->enable_baro != 0);
    cc_runtime.enable_hum  = (file_config->enable_hum  != 0);
    cc_runtime.enable_imu  = (file_config->enable_imu  != 0);
    cc_runtime.enable_mag  = (file_config->enable_mag  != 0);

    cc_runtime.revision = 0;

    CC_RecomputeBudget();
}

const CC_RuntimeConfig_t *CC_Get(void)
{
    return &cc_runtime;
}

/* ── Mutation API ────────────────────────────────────────────────────────── */

bool CC_SetGnssRateMs(uint16_t rate_ms, CC_Source_t source)
{
    if (rate_ms < 40u || rate_ms > 1000u)
    {
        cc_runtime.warning_flags |= CC_WARN_GNSS_RATE_CLAMPED;
        return false;
    }
    cc_runtime.gnss_rate_ms.requested = rate_ms;
    cc_runtime.gnss_rate_ms.effective = rate_ms;
    cc_runtime.gnss_rate_ms.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetBaroOdr(uint8_t odr, CC_Source_t source)
{
    if (odr >= BARO_ODR_TABLE_SIZE) return false;
    cc_runtime.baro_odr.requested = odr;
    cc_runtime.baro_odr.effective = odr;
    cc_runtime.baro_odr.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetHumOdr(uint8_t odr, CC_Source_t source)
{
    if (odr >= HUM_ODR_TABLE_SIZE) return false;
    cc_runtime.hum_odr.requested = odr;
    cc_runtime.hum_odr.effective = odr;
    cc_runtime.hum_odr.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetAccelOdr(uint8_t odr, CC_Source_t source)
{
    if (odr >= ACCEL_ODR_TABLE_SIZE) return false;
    cc_runtime.accel_odr.requested = odr;
    cc_runtime.accel_odr.effective = odr;
    cc_runtime.accel_odr.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetGyroOdr(uint8_t odr, CC_Source_t source)
{
    if (odr >= GYRO_ODR_TABLE_SIZE) return false;
    cc_runtime.gyro_odr.requested = odr;
    cc_runtime.gyro_odr.effective = odr;
    cc_runtime.gyro_odr.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetMagOdr(uint8_t odr, CC_Source_t source)
{
    if (odr >= MAG_ODR_TABLE_SIZE) return false;
    cc_runtime.mag_odr.requested = odr;
    cc_runtime.mag_odr.effective = odr;
    cc_runtime.mag_odr.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetBleDivider(uint8_t sensor_id, uint16_t divider, CC_Source_t source)
{
    CC_U16Field_t *field;
    switch (sensor_id)
    {
        case CC_SENSOR_BARO:  field = &cc_runtime.ble_baro_divider;  break;
        case CC_SENSOR_HUM:   field = &cc_runtime.ble_hum_divider;   break;
        case CC_SENSOR_ACCEL: field = &cc_runtime.ble_accel_divider; break;
        case CC_SENSOR_GYRO:  field = &cc_runtime.ble_gyro_divider;  break;
        case CC_SENSOR_MAG:   field = &cc_runtime.ble_mag_divider;   break;
        default: return false;
    }
    field->requested = divider;
    field->source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetAlRateMs(uint16_t rate_ms, CC_Source_t source)
{
    if (rate_ms < 100u)
    {
        cc_runtime.warning_flags |= CC_WARN_AL_RATE_TOO_FAST;
        return false;
    }
    cc_runtime.al_rate_ms.requested = rate_ms;
    cc_runtime.al_rate_ms.effective = rate_ms;
    cc_runtime.al_rate_ms.source    = source;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}

bool CC_SetAlEnabled(bool enabled, CC_Source_t source)
{
    (void)source; /* No field-level source tracking for the bool flag */
    cc_runtime.al_enabled = enabled;
    CC_RecomputeBudget();
    cc_notify();
    return true;
}
/* ── BLE divider application ──────────────────────────────────────────────── */

static void CC_ApplyBleDividers(void)
{
    BARO_BLE_SetDivider(cc_runtime.ble_baro_divider.effective);
    HUM_BLE_SetDivider(cc_runtime.ble_hum_divider.effective);
    ACCEL_BLE_SetDivider(cc_runtime.ble_accel_divider.effective);
    GYRO_BLE_SetDivider(cc_runtime.ble_gyro_divider.effective);
    MAG_BLE_SetDivider(cc_runtime.ble_mag_divider.effective);
}
