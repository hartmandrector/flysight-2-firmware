# ActiveLook System Overview

## Overview

ActiveLook is a display integration that runs when `al_mode` is enabled in the current configuration. It is not a single user command or a single payload struct. Instead, it is a small state machine that:

1. discovers the ActiveLook glasses over BLE,
2. pushes a configuration sequence to the glasses,
3. builds the selected display layout from config values, and
4. refreshes the page periodically with live GNSS, battery, and sensor-derived values.

The current implementation is split across:

- [FlySight/activelook.c](../FlySight/activelook.c)
- [FlySight/activelook_mode0.c](../FlySight/activelook_mode0.c)
- [STM32_WPAN/App/activelook_client.c](../STM32_WPAN/App/activelook_client.c)
- [FlySight/active_mode.c](../FlySight/active_mode.c)
- [FlySight/config.c](../FlySight/config.c)

## System Diagram

```mermaid
flowchart TD
    A[Active mode starts] --> B{al_mode enabled?}
    B -- no --> Z[No ActiveLook activity]
    B -- yes --> C[FS_ActiveLook_Init()]

    C --> D[Register sequencer task]
    C --> E[Create periodic timer]
    C --> F[Register discovery callback]

    E --> G[Timer fires when READY]
    G --> H[Set FS_TASK_FS_ACTIVELOOK task]

    F --> I[ActiveLook client discovers Rx characteristic]
    I --> J[OnDiscoveryComplete()]
    J --> K[State = CFG_WRITE]
    K --> L[Write cfgWrite packet]
    L --> M[Select mode from config.al_mode]
    M --> N[Mode init/setup sequence]
    N --> O[Write cfgSet packet]
    O --> P[Write clear packet]
    P --> Q[State = READY]

    Q --> R[Periodic update task]
    R --> S[FS_ActiveLook_Mode0_Update()]
    S --> T[Read GNSS / VBAT / battery level / config]
    T --> U[Build one display update packet]
    U --> V[WriteWithoutResp to glasses]
    V --> Q
```

## Runtime Flow

### 1. Entry

ActiveLook is enabled during active mode initialization in [FlySight/active_mode.c](../FlySight/active_mode.c). If `FS_Config_Get()->al_mode != 0`, the ActiveLook subsystem is initialized.

### 2. Discovery and Setup

`FS_ActiveLook_Init()` registers a sequencer task and creates a repeating timer in [FlySight/activelook.c](../FlySight/activelook.c). The ActiveLook client then performs BLE discovery in [STM32_WPAN/App/activelook_client.c](../STM32_WPAN/App/activelook_client.c).

When discovery completes, the callback moves the state machine into setup mode. The setup sequence sends multiple short BLE write-without-response commands rather than one large packet. These include:

- `cfgWrite`
- mode selection
- layout creation
- page creation
- clear/display preparation

### 3. Periodic Updates

After setup, the system sits in `AL_STATE_READY`. The periodic timer only schedules an update when the state machine is ready. The timer does not itself build packets; it just queues the sequencer task.

The actual update work is done by `FS_ActiveLook_Mode0_Update()` in [FlySight/activelook_mode0.c](../FlySight/activelook_mode0.c), which:

- reads current GNSS data,
- reads current VBAT data,
- reads the ActiveLook battery percentage,
- pulls line definitions from config,
- formats up to four text lines,
- sends one page update packet.

## Data Sources

ActiveLook display content comes from several live sources:

- GNSS values from `FS_GNSS_GetData()`
- VBAT values from `FS_VBAT_GetData()`
- ActiveLook battery level from `FS_ActiveLook_Client_GetBatteryLevel()`
- layout and content choices from `FS_Config_Get()`

So although the final BLE packet is a single update message, the display is not driven by a single runtime struct.

## Limits

### Line Count

The layout currently supports up to four ActiveLook lines:

- `FS_CONFIG_MAX_AL_LINES = 4` in [FlySight/config.h](../FlySight/config.h)
- `s_lineSpecs[4]` in [FlySight/activelook_mode0.c](../FlySight/activelook_mode0.c)

### Update Rate

The refresh cadence is controlled by `AL_Rate` in `config.txt` and parsed in [FlySight/config.c](../FlySight/config.c). The current code enforces a minimum value of 100.

The timer starts using `FS_Config_Get()->al_rate` in [FlySight/activelook.c](../FlySight/activelook.c), so the display cadence is user-configurable but bounded.

### Packet Size

The update path sends one display-update packet per refresh. The code builds the packet in memory first, then sends it using `FS_ActiveLook_Client_WriteWithoutResp()`. The display content is therefore limited by the current packet format and the BLE transport characteristics, not by a bulk stream mechanism.

## Important Behavior

- Setup is stateful and incremental; it is not one monolithic BLE write.
- Runtime updates are periodic, not continuously pushed every time GNSS changes.
- The mode content is selected from config and mapped through `activelook_mode0.c`.
- The display uses current live values at update time, so it reflects the latest GNSS and battery state when the timer fires.

## Sequence Summary

1. Active mode starts.
2. ActiveLook is enabled from config.
3. The glasses are discovered.
4. Layouts are configured.
5. The display is cleared and moved to ready state.
6. A repeating timer schedules updates.
7. Each update reads current data and sends one page refresh packet.

## Notes For Future Work

If we later add more ActiveLook modes or more display lines, the best place to extend the architecture is:

- `activelook.c` for the state machine,
- `activelook_mode0.c` for content composition,
- `config.c` for user-facing rate and line settings,
- `activelook_client.c` for BLE discovery and transport details.
