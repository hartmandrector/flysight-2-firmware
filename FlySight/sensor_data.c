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

#include "sensor_data.h"
#include "control_point_protocol.h"
#include "custom_stm.h"       // For CUSTOM_STM_SD_CONTROL_POINT char opcode
#include "ble_tx_queue.h"     // For BLE_TX_Queue_SendTxPacket
#include "gnss_ble.h"         // For GNSS_BLE_SetMask, GNSS_BLE_GetMask
#include "baro_ble.h"
#include "hum_ble.h"
#include "accel_ble.h"
#include "gyro_ble.h"
#include "mag_ble.h"
#include "fusion_integration.h" // For Fusion calibration setters
#include "ble_config.h"       // For validation and auto-calculation
#include "config.h"           // For FS_Config_Get
#include "current_config.h"   // For CC_SetGnssRateMs, CC_SetBleDivider, CC_Get
#include "gnss.h"             // For FS_GNSS_SetDynamicModel, FS_GNSS_SetRateMs
#include "string.h"           // For memcpy
#include "app_common.h"       // For APP_DBG_MSG (optional)
#include "dbg_trace.h"

extern uint8_t SizeSd_Control_Point;

void SensorData_Init(void)
{
	  GNSS_BLE_Init();
}

/* Apply all effective BLE dividers from CC to the streaming modules.
 * Must be called after any CC mutation that can change effective dividers
 * (GNSS rate change, divider set) so hardware state stays in sync with CC. */
static void cc_apply_ble_dividers(void)
{
    const CC_RuntimeConfig_t *cc = CC_Get();
    BARO_BLE_SetDivider(cc->ble_baro_divider.effective);
    HUM_BLE_SetDivider(cc->ble_hum_divider.effective);
    ACCEL_BLE_SetDivider(cc->ble_accel_divider.effective);
    GYRO_BLE_SetDivider(cc->ble_gyro_divider.effective);
    MAG_BLE_SetDivider(cc->ble_mag_divider.effective);
}

/* Build and send the Current Config snapshot as a sequence of BLE notifications
 * on CUSTOM_STM_SD_CONTROL_POINT.
 *
 * Framing:
 *   Packet 1 : [0xF0][0x30][SUCCESS][total_len:u8][16 bytes payload]
 *   Packets 2+: [0xF1][seq:u8][up to 18 bytes payload]
 *
 * Payload (82 bytes, all little-endian):
 *   revision(u32)
 *   gnss_rate_ms / baro_odr / hum_odr / accel_odr / gyro_odr / mag_odr
 *     / ble_baro_div / ble_hum_div / ble_accel_div / ble_gyro_div / ble_mag_div
 *     / al_rate_ms    → each field: requested(u16) effective(u16) source(u8) = 5 bytes
 *   al_enabled(u8)
 *   ble_estimated_bytes_per_sec(u32)  ble_sensor_bytes_per_sec(u32)
 *   ble_activelook_bytes_per_sec(u32)  ble_budget_ok(u8)  warning_flags(u32)
 */
#define CC_BLE_SNAPSHOT_LEN   82u
#define CC_SNAP_FIRST_DATA    16u   /* payload bytes in packet 1 (after 4-byte header) */
#define CC_SNAP_CONT_DATA     18u   /* payload bytes per continuation packet           */

static void send_cc_snapshot(void)
{
    const CC_RuntimeConfig_t *cc = CC_Get();
    uint8_t payload[CC_BLE_SNAPSHOT_LEN];
    uint8_t *p = payload;

#define PACK_U16(v) do { *p++ = (uint8_t)(v); *p++ = (uint8_t)((v) >> 8); } while (0)
#define PACK_U32(v) do { *p++ = (uint8_t)(v);       *p++ = (uint8_t)((v) >> 8); \
                         *p++ = (uint8_t)((v) >> 16); *p++ = (uint8_t)((v) >> 24); } while (0)
#define PACK_FIELD(f) do { PACK_U16((f).requested); PACK_U16((f).effective); *p++ = (f).source; } while (0)

    PACK_U32(cc->revision);
    PACK_FIELD(cc->gnss_rate_ms);
    PACK_FIELD(cc->baro_odr);
    PACK_FIELD(cc->hum_odr);
    PACK_FIELD(cc->accel_odr);
    PACK_FIELD(cc->gyro_odr);
    PACK_FIELD(cc->mag_odr);
    PACK_FIELD(cc->ble_baro_divider);
    PACK_FIELD(cc->ble_hum_divider);
    PACK_FIELD(cc->ble_accel_divider);
    PACK_FIELD(cc->ble_gyro_divider);
    PACK_FIELD(cc->ble_mag_divider);
    PACK_FIELD(cc->al_rate_ms);
    *p++ = cc->al_enabled ? 1u : 0u;
    PACK_U32(cc->ble_estimated_bytes_per_sec);
    PACK_U32(cc->ble_sensor_bytes_per_sec);
    PACK_U32(cc->ble_activelook_bytes_per_sec);
    *p++ = cc->ble_budget_ok ? 1u : 0u;
    PACK_U32(cc->warning_flags);

#undef PACK_FIELD
#undef PACK_U32
#undef PACK_U16

    /* First packet: 4-byte header + CC_SNAP_FIRST_DATA bytes of payload */
    uint8_t pkt[20];
    pkt[0] = CP_RESPONSE_ID;
    pkt[1] = SD_CMD_GET_CURRENT_CONFIG;
    pkt[2] = CP_STATUS_SUCCESS;
    pkt[3] = (uint8_t)CC_BLE_SNAPSHOT_LEN;
    memcpy(&pkt[4], &payload[0], CC_SNAP_FIRST_DATA);
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_CONTROL_POINT,
                              pkt, 4u + CC_SNAP_FIRST_DATA,
                              &SizeSd_Control_Point, 0);

    /* Continuation packets: [0xF1][seq][up to CC_SNAP_CONT_DATA bytes] */
    uint8_t offset = CC_SNAP_FIRST_DATA;
    uint8_t seq    = 1u;
    while (offset < CC_BLE_SNAPSHOT_LEN) {
        uint8_t chunk = CC_BLE_SNAPSHOT_LEN - offset;
        if (chunk > CC_SNAP_CONT_DATA) chunk = CC_SNAP_CONT_DATA;
        pkt[0] = CP_CONTINUATION_ID;
        pkt[1] = seq++;
        memcpy(&pkt[2], &payload[offset], chunk);
        BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_CONTROL_POINT,
                                  pkt, (uint8_t)(2u + chunk),
                                  &SizeSd_Control_Point, 0);
        offset += chunk;
    }
}

void SensorData_Handle_SD_ControlPointWrite(
		const uint8_t *payload, uint8_t length,
		uint16_t conn_handle, uint8_t notification_enabled_flag)
{
    (void)conn_handle; // Mark as unused if not needed by this handler specifically

    uint8_t received_cmd_opcode = 0xFF; // Default for error case / no valid command
    uint8_t status = CP_STATUS_ERROR_UNKNOWN;
    uint8_t response_data_buf[MAX_CP_OPTIONAL_RESPONSE_DATA_LEN] = {0};
    uint8_t response_data_len = 0;
    bool response_sent = false;

    if (length < 1)
    {
        status = CP_STATUS_INVALID_PARAMETER;
        // received_cmd_opcode remains 0xFF (or set to a specific "invalid command" opcode if you define one)
    }
    else
    {
        received_cmd_opcode = payload[0];
        const uint8_t *params = &payload[1];
        uint8_t params_len = length - 1;

        switch (received_cmd_opcode)
        {
            case SD_CMD_SET_GNSS_BLE_MASK:
                if (params_len != 1) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    GNSS_BLE_SetMask(params[0]); // Directly call the utility in gnss_ble.c
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case SD_CMD_GET_GNSS_BLE_MASK:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    response_data_buf[0] = GNSS_BLE_GetMask(); // Call utility
                    response_data_len = 1;
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case SD_CMD_SET_BLE_DIVIDER:
                if (params_len != 3) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    uint8_t sensor_id = params[0];
                    uint16_t divider = params[1] | (params[2] << 8); // Little-endian
                    /* CC_SetBleDivider validates sensor_id, updates requested value,
                     * recomputes budget (recalculating all auto dividers), and
                     * increments revision. Then we apply all effective dividers to
                     * the BLE streaming modules so auto peers update too. */
                    if (!CC_SetBleDivider(sensor_id, divider, CC_SRC_CONTROL_POINT)) {
                        status = CP_STATUS_INVALID_PARAMETER;
                    } else {
                        cc_apply_ble_dividers();
                        status = CP_STATUS_SUCCESS;
                    }
                }
                break;

            case SD_CMD_GET_BLE_DIVIDER:
                if (params_len != 1) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    uint8_t sensor_id = params[0];
                    const CC_RuntimeConfig_t *cc = CC_Get();
                    uint16_t divider = 0;
                    switch (sensor_id) {
                        case CC_SENSOR_BARO:  divider = cc->ble_baro_divider.effective;  status = CP_STATUS_SUCCESS; break;
                        case CC_SENSOR_HUM:   divider = cc->ble_hum_divider.effective;   status = CP_STATUS_SUCCESS; break;
                        case CC_SENSOR_ACCEL: divider = cc->ble_accel_divider.effective; status = CP_STATUS_SUCCESS; break;
                        case CC_SENSOR_GYRO:  divider = cc->ble_gyro_divider.effective;  status = CP_STATUS_SUCCESS; break;
                        case CC_SENSOR_MAG:   divider = cc->ble_mag_divider.effective;   status = CP_STATUS_SUCCESS; break;
                        default: status = CP_STATUS_INVALID_PARAMETER; break;
                    }
                    if (status == CP_STATUS_SUCCESS) {
                        response_data_buf[0] = sensor_id;
                        response_data_buf[1] = divider & 0xFF;        // Low byte
                        response_data_buf[2] = (divider >> 8) & 0xFF; // High byte
                        response_data_len = 3;
                    }
                }
                break;

            case SD_CMD_SET_GNSS_MODEL:
                if (params_len != 1) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    uint8_t model = params[0];
                    bool valid_model = (model == FS_CONFIG_MODEL_PORTABLE) ||
                                       (model == FS_CONFIG_MODEL_STATIONARY) ||
                                       (model == FS_CONFIG_MODEL_PEDESTRIAN) ||
                                       (model == FS_CONFIG_MODEL_AUTOMOTIVE) ||
                                       (model == FS_CONFIG_MODEL_SEA) ||
                                       (model == FS_CONFIG_MODEL_AIRBORNE_1G) ||
                                       (model == FS_CONFIG_MODEL_AIRBORNE_2G) ||
                                       (model == FS_CONFIG_MODEL_AIRBORNE_4G);

                    if (!valid_model) {
                        status = CP_STATUS_INVALID_PARAMETER;
                    } else if (FS_GNSS_SetDynamicModel(model) != HAL_OK) {
                        status = CP_STATUS_OPERATION_FAILED;
                    } else {
                        status = CP_STATUS_SUCCESS;
                    }
                }
                break;

            case SD_CMD_SET_GNSS_RATE:
                if (params_len != 2) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    uint16_t rate_ms = (uint16_t) params[0] |
                                       ((uint16_t) params[1] << 8);
                    if (!CC_SetGnssRateMs(rate_ms, CC_SRC_CONTROL_POINT)) {
                        status = CP_STATUS_INVALID_PARAMETER;
                    } else if (FS_GNSS_SetRateMs(rate_ms) != HAL_OK) {
                        status = CP_STATUS_OPERATION_FAILED;
                    } else {
                        /* GNSS rate change recomputes auto dividers in CC;
                         * propagate updated effective values to BLE modules. */
                        cc_apply_ble_dividers();
                        status = CP_STATUS_SUCCESS;
                    }
                }
                break;

            case SD_CMD_SET_FUSION_MAG_HARD:
                // Payload: 6 bytes (3 × int16_t milligauss, little-endian)
                if (params_len != 6) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    int16_t x_mg = (int16_t)(params[0] | (params[1] << 8));
                    int16_t y_mg = (int16_t)(params[2] | (params[3] << 8));
                    int16_t z_mg = (int16_t)(params[4] | (params[5] << 8));
                    
                    // Convert milligauss to gauss for Fusion
                    FusionVector hard_iron = {
                        .axis = {
                            .x = (float)x_mg / 1000.0f,
                            .y = (float)y_mg / 1000.0f,
                            .z = (float)z_mg / 1000.0f
                        }
                    };
                    
                    FS_Fusion_SetMagHardIron(hard_iron);
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case SD_CMD_SET_FUSION_MAG_SOFT:
                // Payload: 36 bytes (9 × int32_t scaled by 1000000, little-endian)
                if (params_len != 36) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    FusionMatrix soft_iron;
                    
                    // Parse 9 matrix elements (row-major order)
                    for (int i = 0; i < 9; i++) {
                        int32_t val = (int32_t)(params[i*4 + 0] |
                                               (params[i*4 + 1] << 8) |
                                               (params[i*4 + 2] << 16) |
                                               (params[i*4 + 3] << 24));
                        float f_val = (float)val / 1000000.0f;
                        
                        // Assign to matrix (row-major)
                        switch(i) {
                            case 0: soft_iron.element.xx = f_val; break;
                            case 1: soft_iron.element.xy = f_val; break;
                            case 2: soft_iron.element.xz = f_val; break;
                            case 3: soft_iron.element.yx = f_val; break;
                            case 4: soft_iron.element.yy = f_val; break;
                            case 5: soft_iron.element.yz = f_val; break;
                            case 6: soft_iron.element.zx = f_val; break;
                            case 7: soft_iron.element.zy = f_val; break;
                            case 8: soft_iron.element.zz = f_val; break;
                        }
                    }
                    
                    FS_Fusion_SetMagSoftIron(soft_iron);
                    status = CP_STATUS_SUCCESS;
                }
                break;

            case SD_CMD_GET_CURRENT_CONFIG:
                if (params_len != 0) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else if (notification_enabled_flag) {
                    send_cc_snapshot();
                    response_sent = true;
                }
                break;

            default:
                status = CP_STATUS_CMD_NOT_SUPPORTED;
                break;
        }
    }

    if (notification_enabled_flag && !response_sent)
    {
        uint8_t final_response_packet[3 + MAX_CP_OPTIONAL_RESPONSE_DATA_LEN];
        uint8_t total_response_len = 3;

        final_response_packet[0] = CP_RESPONSE_ID;
        final_response_packet[1] = received_cmd_opcode; // Echo original or 0xFF
        final_response_packet[2] = status;

        if (status == CP_STATUS_SUCCESS && response_data_len > 0)
        {
            if (response_data_len <= MAX_CP_OPTIONAL_RESPONSE_DATA_LEN)
            {
                memcpy(&final_response_packet[3], response_data_buf, response_data_len);
                total_response_len += response_data_len;
            }
            else
            {
                // This case should ideally be prevented by MAX_CP_OPTIONAL_RESPONSE_DATA_LEN
                // Send error or success without data. For robustness, an error might be better.
                final_response_packet[2] = CP_STATUS_ERROR_UNKNOWN; // Override status
                response_data_len = 0; // Don't copy data
                total_response_len = 3;
                APP_DBG_MSG("SensorData: Response data too long for cmd %02X\n", received_cmd_opcode);
            }
        }

        SizeSd_Control_Point = total_response_len; // Update the global characteristic size
        BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_CONTROL_POINT,
                                  final_response_packet,
                                  total_response_len,
                                  &SizeSd_Control_Point, // Pass its address
                                  0);
    }
}
