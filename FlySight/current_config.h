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

#ifndef CURRENT_CONFIG_H_
#define CURRENT_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

/*
 * Current Config — runtime source of truth
 *
 * Tracks every user-configurable rate and divider so all calculations
 * always operate on live runtime values rather than potentially stale
 * file-config values.
 *
 * See Docs/CURRENT_CONFIG_DESIGN.md for the full architecture spec.
 */

/* ── Value provenance ──────────────────────────────────────────────────── */

typedef uint8_t CC_Source_t;
#define CC_SRC_DEFAULT        0u   /* Compiled-in default */
#define CC_SRC_FILE           1u   /* Set by config.txt at boot */
#define CC_SRC_CONTROL_POINT  2u   /* Changed by BLE control point at runtime */
#define CC_SRC_AUTO_CALC      3u   /* Computed by BLE budget algorithm */

/* ── Persist policy ────────────────────────────────────────────────────── */

typedef uint8_t CC_PersistPolicy_t;
#define CC_PERSIST_NEVER       0u   /* Runtime-only; not written back to config file */
#define CC_PERSIST_ON_REQUEST  1u   /* Written to file only on explicit save request */

/* ── Per-field wrapper ─────────────────────────────────────────────────── */

typedef struct {
    uint16_t           requested; /* What was asked for (0 = auto for dividers) */
    uint16_t           effective; /* What is actually in use */
    CC_Source_t        source;    /* Where the effective value came from */
    CC_PersistPolicy_t persist;   /* Save policy */
} CC_U16Field_t;

/* ── Warning flag bits ─────────────────────────────────────────────────── */

#define CC_WARN_BLE_OVER_BUDGET        (1u << 0) /* Total BLE BW exceeds safe limit */
#define CC_WARN_GNSS_RATE_CLAMPED      (1u << 1) /* GNSS rate was out of range and clamped */
#define CC_WARN_DIVIDER_AT_MIN_RATE    (1u << 2) /* At least one divider is at 0.5 Hz floor */
#define CC_WARN_AL_RATE_TOO_FAST       (1u << 3) /* al_rate_ms < 100 — rejected */
#define CC_WARN_AL_HIGH_BW             (1u << 4) /* ActiveLook consuming >30 % of budget */

/* ── Top-level runtime configuration ───────────────────────────────────── */

typedef struct {
    uint32_t revision;           /* Incremented on every mutation */

    /* GNSS */
    CC_U16Field_t gnss_rate_ms;  /* Measurement interval in ms (40–1000) */

    /* Sensor ODRs (config-file indices, not Hz) */
    CC_U16Field_t baro_odr;
    CC_U16Field_t hum_odr;
    CC_U16Field_t accel_odr;
    CC_U16Field_t gyro_odr;
    CC_U16Field_t mag_odr;

    /* BLE transmit dividers (0 requested = auto-calculate) */
    CC_U16Field_t ble_baro_divider;
    CC_U16Field_t ble_hum_divider;
    CC_U16Field_t ble_accel_divider;
    CC_U16Field_t ble_gyro_divider;
    CC_U16Field_t ble_mag_divider;

    /* ActiveLook */
    CC_U16Field_t al_rate_ms;    /* Display refresh interval in ms (min 100) */
    bool          al_enabled;    /* true when al_mode != 0 */

    /* BLE budget summary — read-only outputs, updated by CC_RecomputeBudget() */
    uint32_t ble_estimated_bytes_per_sec;    /* GPS + AL + all sensors */
    uint32_t ble_sensor_bytes_per_sec;       /* Sensor traffic only */
    uint32_t ble_activelook_bytes_per_sec;   /* ActiveLook traffic only */
    bool     ble_budget_ok;

    /* Enable flags */
    bool enable_gnss;
    bool enable_baro;
    bool enable_hum;
    bool enable_imu;
    bool enable_mag;

    /* Validation warnings (bitmask; cleared on each CC_RecomputeBudget call) */
    uint32_t warning_flags;
} CC_RuntimeConfig_t;

/* ── Sensor IDs for CC_SetBleDivider ───────────────────────────────────── */

#define CC_SENSOR_BARO   0u
#define CC_SENSOR_HUM    1u
#define CC_SENSOR_ACCEL  2u
#define CC_SENSOR_GYRO   3u
#define CC_SENSOR_MAG    4u

/* ── API ────────────────────────────────────────────────────────────────── */

/* Lifecycle */

/* Seed from compiled defaults then overlay file config values.
 * Call once after all FS_Config_Read() calls have completed. */
void CC_Init(const FS_Config_Data_t *file_config);

/* Read-only snapshot — never returns NULL after CC_Init(). */
const CC_RuntimeConfig_t *CC_Get(void);

/* Mutation — each call validates, updates, recomputes budget, and
 * invokes registered callbacks.  Returns false if validation fails. */
bool CC_SetGnssRateMs(uint16_t rate_ms, CC_Source_t source);
bool CC_SetBaroOdr(uint8_t odr, CC_Source_t source);
bool CC_SetHumOdr(uint8_t odr, CC_Source_t source);
bool CC_SetAccelOdr(uint8_t odr, CC_Source_t source);
bool CC_SetGyroOdr(uint8_t odr, CC_Source_t source);
bool CC_SetMagOdr(uint8_t odr, CC_Source_t source);
bool CC_SetBleDivider(uint8_t sensor_id, uint16_t divider, CC_Source_t source);
bool CC_SetAlRateMs(uint16_t rate_ms, CC_Source_t source);
bool CC_SetAlEnabled(bool enabled, CC_Source_t source);

/* Force a full BLE budget recompute without changing any input field.
 * Useful after enable flags change. */
void CC_RecomputeBudget(void);

/* Persistence */

/* Fill out_config with current effective values so FS_Config_Write() can
 * persist a fully-reproducible snapshot. */
void CC_ExportToFileConfig(FS_Config_Data_t *out_config);

/* Callbacks — invoked after every mutation + recompute */
typedef void (*CC_ChangeCallback_t)(const CC_RuntimeConfig_t *new_cfg);
void CC_RegisterChangeCallback(CC_ChangeCallback_t cb);

#endif /* CURRENT_CONFIG_H_ */
