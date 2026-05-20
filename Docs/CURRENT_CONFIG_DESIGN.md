# Current Running Configuration — Architecture Design Spec

## Overview

FlySight 2 firmware has three distinct layers that contribute to system behaviour at runtime:

1. **Compiled defaults** — `FS_Config_Init()` in `config.c`
2. **Persistent config file** — `config.txt` parsed by `FS_Config_Read()`
3. **Runtime mutations** — control point commands (`sensor_data.c`)

Currently there is no single authoritative source that reflects **what the device is actually doing right now**. Modules such as `ble_config.c` read from the persistent config struct even after control point overrides have changed hardware state. This creates configuration drift, makes BLE budget calculations incorrect after runtime GNSS/ODR changes, and makes exhaustive mode testing non-deterministic.

Two additional gaps have been identified since the initial design:

- **Control-point GPS rate changes** — The GNSS rate can be altered at runtime via `SD_CMD_SET_GNSS_RATE`. The BLE budget algorithm in `ble_config.c` still reads `config->rate` (the file value), so a runtime rate change is invisible to the budget calculator until reboot.
- **ActiveLook BLE bandwidth** — When `al_mode` is enabled, the firmware opens a separate BLE central connection to the ActiveLook glasses and sends periodic `pageClearAndDisplay` packets at the configured `al_rate`. This traffic competes for radio time on the same STM32WB radio used for sensor streaming, but is currently not included in the BLE budget calculation at all.

This document specifies a **Current Config** service that becomes the single runtime source of truth, tracking every user-configurable rate and divider — including the GNSS rate as set at runtime and the ActiveLook update cadence — so that the BLE budget algorithm always operates on accurate data.

---

## Goals

- One place that reflects the device's actual running configuration at any moment.
- All rate/budget calculations consume current runtime values, not potentially stale file config.
- Control point mutations are submitted through a validated, dependency-aware pipeline.
- Support for exporting the current effective configuration for testing and debugging.
- Incremental migration — no flag-day rewrite.

---

## Key Concepts

### Value Provenance

Each tunable field has a **source** tag indicating where its current value came from:

| Source | Meaning |
|---|---|
| `CC_SRC_DEFAULT` | Value comes from compiled defaults |
| `CC_SRC_FILE` | Value was overridden by `config.txt` at boot |
| `CC_SRC_CONTROL_POINT` | Value was changed by BLE control point command at runtime |
| `CC_SRC_AUTO_CALC` | Value was computed by an internal algorithm (e.g. BLE budget) |

### Requested vs. Effective

Some fields have two values:

- **Requested** — what the caller/user specified. May be `0` (auto) or a concrete value.
- **Effective** — the actual value the system is running with, after auto-calculation or clamping.

Example: `ble_accel_divider`
- Requested: `0` (auto)
- Effective: `28` (computed from ODR 416 Hz and remaining budget)

All hardware apply calls and transmit-loop decimation use **effective** values.
All export and GET control point responses return **both** values so callers can distinguish.

### Persist Policy

Each field has a persist policy:

| Policy | Meaning |
|---|---|
| `CC_PERSIST_NEVER` | Runtime-only; never written back to config file |
| `CC_PERSIST_ON_REQUEST` | Written to config file only when user explicitly requests a save |

Runtime GNSS rate changes and divider overrides default to `CC_PERSIST_NEVER`.

---

## Data Model

### Runtime Configuration Struct

```c
// FlySight/current_config.h

typedef uint8_t CC_Source_t;
#define CC_SRC_DEFAULT        0
#define CC_SRC_FILE           1
#define CC_SRC_CONTROL_POINT  2
#define CC_SRC_AUTO_CALC      3

typedef uint8_t CC_PersistPolicy_t;
#define CC_PERSIST_NEVER       0
#define CC_PERSIST_ON_REQUEST  1

// Per-field metadata wrapper (for the fields that need tracking)
typedef struct {
    uint16_t requested;          // What was asked for (0 = auto)
    uint16_t effective;          // What is actually in use
    CC_Source_t source;          // Where the effective value came from
    CC_PersistPolicy_t persist;  // Save policy
} CC_U16Field_t;

// Top-level runtime configuration
typedef struct {
    uint32_t revision;           // Incremented on every change; useful for test assertions

    // GNSS
    CC_U16Field_t gnss_rate_ms;  // Measurement interval (40–1000 ms)

    // Sensor ODRs (requested only; effective = same unless clamped)
    CC_U16Field_t baro_odr;
    CC_U16Field_t hum_odr;
    CC_U16Field_t accel_odr;
    CC_U16Field_t gyro_odr;
    CC_U16Field_t mag_odr;

    // BLE dividers
    CC_U16Field_t ble_baro_divider;
    CC_U16Field_t ble_hum_divider;
    CC_U16Field_t ble_accel_divider;
    CC_U16Field_t ble_gyro_divider;
    CC_U16Field_t ble_mag_divider;

    // ActiveLook
    // al_rate_ms: interval between display refreshes (100–65535 ms; 0 = not configured).
    // Seeded from config->al_rate at boot; may be overridden by a future control point command.
    // al_enabled: derived from config->al_mode != 0; gates the AL bandwidth slice in budget calc.
    CC_U16Field_t al_rate_ms;        // Refresh interval in ms (min 100)
    bool          al_enabled;        // true when al_mode != 0

    // BLE budget summary (read-only outputs, CC_SRC_AUTO_CALC)
    // ble_estimated_bytes_per_sec includes all consumers: GPS, ActiveLook, and sensors.
    // ble_sensor_bytes_per_sec covers sensors only (useful for diagnosing AL vs sensor load).
    // ble_activelook_bytes_per_sec covers the AL central-connection periodic update traffic.
    //
    // NOTE: ActiveLook traffic is on a separate BLE central connection but shares the same
    // STM32WB radio.  It is modelled as a flat bandwidth subtraction from the shared budget.
    uint32_t ble_estimated_bytes_per_sec;
    uint32_t ble_sensor_bytes_per_sec;
    uint32_t ble_activelook_bytes_per_sec;
    bool     ble_budget_ok;

    // Enable flags (runtime-mutable in future phases; read from file config for now)
    bool enable_gnss;
    bool enable_baro;
    bool enable_hum;
    bool enable_imu;
    bool enable_mag;

    // Validation warnings (bitmask; cleared on each recompute)
    uint32_t warning_flags;
} CC_RuntimeConfig_t;

// Warning flag bits
#define CC_WARN_BLE_OVER_BUDGET        (1u << 0)
#define CC_WARN_GNSS_RATE_CLAMPED      (1u << 1)
#define CC_WARN_DIVIDER_AT_MIN_RATE    (1u << 2)
#define CC_WARN_AL_RATE_TOO_FAST       (1u << 3)   // al_rate_ms < 100 — rejected
#define CC_WARN_AL_HIGH_BW             (1u << 4)   // ActiveLook consuming >30 % of budget
```

---

## Module API

```c
// FlySight/current_config.h  (continued)

// ── Lifecycle ────────────────────────────────────────────────────────────────

// Seed from compiled defaults then overlay config file values.
// Call once at boot, before any hardware init.
void CC_Init(const FS_Config_Data_t *file_config);

// Read-only snapshot access.
const CC_RuntimeConfig_t *CC_Get(void);

// ── Mutation API ──────────────────────────────────────────────────────────────
// All mutations:
//   1. Validate the new value.
//   2. Update the field (requested + source).
//   3. Run dependency recomputation.
//   4. Invoke registered hardware apply callbacks.
//   5. Increment revision.
//
// Returns true on success, false if validation fails (value not applied).

bool CC_SetGnssRateMs(uint16_t rate_ms, CC_Source_t source);
bool CC_SetBaroOdr(uint8_t odr, CC_Source_t source);
bool CC_SetHumOdr(uint8_t odr, CC_Source_t source);
bool CC_SetAccelOdr(uint8_t odr, CC_Source_t source);
bool CC_SetGyroOdr(uint8_t odr, CC_Source_t source);
bool CC_SetMagOdr(uint8_t odr, CC_Source_t source);
bool CC_SetBleDivider(uint8_t sensor_id, uint16_t divider, CC_Source_t source);

// ActiveLook rate and enable mutations.
// CC_SetAlRateMs validates rate_ms >= 100; rejects values below the minimum.
// CC_SetAlEnabled seeds al_enabled at boot from al_mode; may be called again at runtime
// when the ActiveLook connection is established or torn down.
// Both trigger a full BLE budget recompute.
bool CC_SetAlRateMs(uint16_t rate_ms, CC_Source_t source);
bool CC_SetAlEnabled(bool enabled, CC_Source_t source);

// Force a full BLE budget recompute without changing any input field.
// Useful after enable flags change.
void CC_RecomputeBudget(void);

// ── Persistence ───────────────────────────────────────────────────────────────

// Write all CC_PERSIST_ON_REQUEST fields back to the config struct
// so FS_Config_Write() can persist them.
void CC_ExportToFileConfig(FS_Config_Data_t *out_config);

// ── Callbacks ─────────────────────────────────────────────────────────────────

// Register a callback invoked whenever a field group changes.
// hardware_apply is called with the new snapshot after recompute.
typedef void (*CC_ChangeCallback_t)(const CC_RuntimeConfig_t *new_cfg);
void CC_RegisterChangeCallback(CC_ChangeCallback_t cb);
```

---

## Dependency Recomputation Rules

These run automatically inside the mutation functions, in order:

| Trigger | Recomputed outputs |
|---|---|
| GNSS rate changes | BLE GPS bandwidth slice → BLE budget → auto dividers |
| ActiveLook rate changes | BLE AL bandwidth slice → BLE budget → auto dividers |
| ActiveLook enabled/disabled | BLE AL bandwidth slice → BLE budget → auto dividers |
| Any sensor ODR changes | Sensor hardware rate → BLE budget slice → auto dividers |
| Any BLE divider changes (manual) | BLE budget (manual slice) → auto divider peers |
| Enable flag changes | Budget participation set → BLE budget → auto dividers |

**Computation order is always:**
1. GPS bandwidth = `(1000 / gnss_rate_ms.effective) × GPS_PACKET_SIZE`
   - `gnss_rate_ms.effective` is the **runtime** value — may have been set by a control point command, not just the file config.
2. ActiveLook bandwidth = `al_enabled ? (1000 / al_rate_ms.effective) × AL_PACKET_SIZE_ESTIMATE : 0`
   - `AL_PACKET_SIZE_ESTIMATE` = 60 bytes (conservative estimate for one `pageClearAndDisplay` packet: 4-byte header + page ID + heading string + up to 4 data strings + footer).
   - ActiveLook uses a separate BLE central connection to the glasses but shares the same STM32WB radio, so its bandwidth is subtracted from the shared budget alongside GPS.
3. Manual sensor bandwidth = sum over sensors where `ble_*_divider.requested != 0`
4. Remaining budget = `BLE_SAFE_THROUGHPUT_LIMIT − GPS_BW − AL_BW − MANUAL_BW`
5. Auto sensor full-ODR bandwidth = sum over sensors where `ble_*_divider.requested == 0`
6. If auto bandwidth ≤ remaining budget → effective divider = 1 for all auto sensors
7. Otherwise → scale all auto dividers by `auto_bandwidth / remaining_budget` (round up)
8. Clamp each divider to `max_divider` (enforces 0.5 Hz minimum)
9. Write results to `ble_*_divider.effective` with `source = CC_SRC_AUTO_CALC`
10. Update `ble_sensor_bytes_per_sec`, `ble_activelook_bytes_per_sec`, `ble_estimated_bytes_per_sec` (= GPS_BW + AL_BW + sensor BW), `ble_budget_ok`, `warning_flags`

This replaces the equivalent logic currently split between `ble_config.c`, `sensor_data.c`, and each `*_ble.c` `Init()` function.

---

## Hardware Apply Callbacks

Each hardware domain registers a callback during module init:

```c
// Example registrations (in respective module init functions)

// gnss.c
CC_RegisterChangeCallback(CC_ApplyGnssRate_Callback);

// imu.c / baro.c / etc.
CC_RegisterChangeCallback(CC_ApplyOdrSettings_Callback);

// custom_app.c (BLE decimation)
CC_RegisterChangeCallback(CC_ApplyBleDividers_Callback);
```

Each callback checks the new snapshot against what it last applied and acts only on changed fields. This avoids redundant hardware writes on every change.

The BLE decimation callback in `custom_app.c` replaces the current pattern of reading directly from `ACCEL_BLE_GetDivider()` etc., or it can simply call the existing `*_BLE_SetDivider()` setters with the new effective values — either works during migration.

---

## Export / Readout Interface

### Control Point Command (future)

Add `SD_CMD_GET_CURRENT_CONFIG` (suggested opcode `0x30`) returning a compact binary snapshot:

```
[revision (uint32)]
[gnss_rate_ms_requested (uint16)] [gnss_rate_ms_effective (uint16)] [source (uint8)]
[ble_baro_divider_requested (uint16)] [ble_baro_divider_effective (uint16)] [source (uint8)]
... (one entry per tracked field)
[ble_estimated_bytes_per_sec (uint32)]
[ble_budget_ok (uint8)]
[warning_flags (uint32)]
```

Because the full snapshot exceeds a single 20-byte ATT packet, this response should be chunked using the existing control point indication mechanism (or queued via `BLE_TX_Queue_SendTxPacket`).

### USB / Debug Dump

`CC_ExportToFileConfig()` can be called at any time to fill a `FS_Config_Data_t` with current effective values. Writing that struct to a file gives you a fully reproducible snapshot for test replay.

---

## Migration Plan

Migration is additive and per-phase. No phase breaks existing behaviour.

### Phase 1 — Introduce Module, Mirror Current State (no behaviour change)

- Create `FlySight/current_config.c` and `FlySight/current_config.h`.
- `CC_Init()` reads from `FS_Config_Data_t` (already parsed) and populates the runtime struct.
  - Seeds `gnss_rate_ms` from `config->rate`.
  - Seeds `al_rate_ms` from `config->al_rate`; seeds `al_enabled` from `config->al_mode != 0`.
- `CC_Get()` returns a read-only snapshot.
- All existing code continues to use `FS_Config_Get()` and module getters unchanged.
- Add `revision` counter incrementing.
- **Test:** Assert that `CC_Get()` values match `FS_Config_Get()` at boot; assert `ble_activelook_bytes_per_sec == 0` when `al_mode == 0`.

### Phase 2 — Route GNSS Rate Mutations and Wire ActiveLook Budget

- `sensor_data.c` `SD_CMD_SET_GNSS_RATE` handler calls `CC_SetGnssRateMs()` **instead of** calling `FS_GNSS_SetRateMs()` directly.
- `CC_SetGnssRateMs()` validates, updates the field with `CC_SRC_CONTROL_POINT`, calls `CC_RecomputeBudget()`, then calls `FS_GNSS_SetRateMs()` via registered callback.
- `ble_config.c` validation and auto-calc read `CC_Get()->gnss_rate_ms.effective` instead of `config->rate`.
- `ble_config.c` auto-calc adds the ActiveLook bandwidth slice (step 2 of the new computation order), reading `CC_Get()->al_enabled` and `CC_Get()->al_rate_ms.effective`.
- `activelook.c` calls `CC_SetAlEnabled(true, CC_SRC_FILE)` during `FS_ActiveLook_Init()` so the budget is always re-evaluated when the ActiveLook subsystem is activated.
- **Test:** Change GNSS rate via control point, verify BLE budget recalculates and sensor dividers adjust. Enable ActiveLook with al_rate=100 ms (10 Hz), verify `ble_activelook_bytes_per_sec ≈ 600` and sensor dividers increase accordingly.

### Phase 3 — Route BLE Divider Mutations

- `sensor_data.c` `SD_CMD_SET_BLE_DIVIDER` handler calls `CC_SetBleDivider()`.
- `CC_SetBleDivider()` triggers recompute, updates effective values, invokes apply callback which calls `*_BLE_SetDivider()`.
- Remove the temp-config + manual sync block currently in `sensor_data.c`.
- **Test:** Mixed manual + auto dividers; verify auto peers recompute correctly. Verify AL bandwidth is preserved across divider changes.

### Phase 4 — Sensor ODR Mutations

- Add `CC_Set*Odr()` for each sensor.
- Any future ODR control point commands go through Current Config.
- ODR changes trigger full budget recompute.

### Phase 5 — Export and Persistence

- Implement `SD_CMD_GET_CURRENT_CONFIG` control point command.
- Implement `CC_ExportToFileConfig()` for USB debug dump and optional save.
- Response includes `ble_activelook_bytes_per_sec` and `al_rate_ms` fields.
- Add `warning_flags` reporting to BLE response.

### Phase 6 — ActiveLook Rate Control Point (future)

- Add a new control point command `SD_CMD_SET_AL_RATE` to allow runtime adjustment of `al_rate_ms`.
- Handler calls `CC_SetAlRateMs(rate_ms, CC_SRC_CONTROL_POINT)`.
- `CC_SetAlRateMs()` validates `rate_ms >= 100`, sets `CC_WARN_AL_RATE_TOO_FAST` and returns false on violation.
- On success: updates `al_rate_ms.effective`, calls `CC_RecomputeBudget()`, then reschedules the ActiveLook timer via registered callback.
- **Test:** Set al_rate=100 ms (max speed), verify budget warning triggers when sensors are at high ODR. Set al_rate=10000 ms (very slow), verify AL bandwidth slice shrinks and sensor dividers relax.

---

## Affected Files by Phase

| Phase | Files Changed | Files Added |
|---|---|---|
| 1 | `active_mode.c`, `start_mode.c` (call CC_Init after all config reads including selectable overlay) | `current_config.c`, `current_config.h` |
| 2 | `sensor_data.c`, `ble_config.c`, `activelook.c` | — |
| 3 | `sensor_data.c`, `custom_app.c` | — |
| 4 | `sensor_data.c` (new ODR commands) | — |
| 5 | `sensor_data.c` (new CP opcode) | — |
| 6 | `sensor_data.c` (new AL rate CP opcode), `activelook.c` (timer reschedule callback) | — |

---

## Open Questions / Decisions Needed

1. **Enable flags** — Should runtime enable/disable of individual sensors go through Current Config in Phase 4 or later? Currently they are set only from the config file.

2. **Fusion parameters** — Fusion calibration mutations (mag hard/soft iron, gain) currently bypass config entirely. Should they be tracked in Current Config too?

3. **Thread safety** — This firmware runs on the ST cooperative sequencer (`UTIL_SEQ`), not a preemptive RTOS. All application code on CPU1 executes cooperatively in a single context; a task switch cannot occur mid-instruction-sequence. Therefore, a `__disable_irq()` / `__enable_irq()` critical section around CC mutations is sufficient to guard against ISR-sourced reads (e.g. timer callbacks reading `al_rate_ms` while a control point handler is writing it). A message queue is not required. Decision: **use critical sections**; implement before Phase 2 goes to hardware.

4. **Chunk size for GET_CURRENT_CONFIG** — Depends on final field count and MTU negotiated with central. May need a multi-packet framing scheme consistent with the file transfer protocol.

5. **Revision rollover** — `uint32_t` revision will not rollover in practice, but test harness should handle it.

6. **ActiveLook packet size estimate** — `AL_PACKET_SIZE_ESTIMATE = 60 bytes` is a conservative estimate based on a typical `pageClearAndDisplay` payload (4-byte framing header + 1-byte page ID + heading string + up to 4 data strings + 1-byte footer). The actual size varies with text content. Two options: (a) keep the fixed estimate and accept mild over- or under-counting, or (b) measure the real packet size in `AL_BuildPageUpdate()` and expose it via `FS_ActiveLook_GetLastPacketSize()` for a more accurate budget input. Option (a) is acceptable for a first implementation.

7. **ActiveLook dual-connection BLE model** — Sensor streaming is on the peripheral connection (phone → FlySight); ActiveLook is on the central connection (FlySight → glasses). Both share the STM32WB radio. The current model subtracts AL bandwidth from a single flat budget. A more precise model would account for BLE scheduling slots per connection interval, but this requires knowledge of the negotiated connection parameters for both links. The flat subtraction model is conservative (may over-throttle sensors slightly) and is the correct starting point.
