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
#include "string.h"           // For memcpy
#include "app_common.h"       // For APP_DBG_MSG (optional)
#include "dbg_trace.h"

extern uint8_t SizeSd_Control_Point;

void SensorData_Init(void)
{
	  GNSS_BLE_Init();
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
                    
                    if (divider == 0) {
                        status = CP_STATUS_INVALID_PARAMETER; // 0 reserved for auto-calc at boot
                    } else if (sensor_id == 0) {
                        BARO_BLE_SetDivider(divider);
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 1) {
                        HUM_BLE_SetDivider(divider);
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 2) {
                        ACCEL_BLE_SetDivider(divider);
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 3) {
                        GYRO_BLE_SetDivider(divider);
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 4) {
                        MAG_BLE_SetDivider(divider);
                        status = CP_STATUS_SUCCESS;
                    } else {
                        status = CP_STATUS_INVALID_PARAMETER; // Invalid sensor_id
                    }
                }
                break;

            case SD_CMD_GET_BLE_DIVIDER:
                if (params_len != 1) {
                    status = CP_STATUS_INVALID_PARAMETER;
                } else {
                    uint8_t sensor_id = params[0];
                    uint16_t divider = 0;
                    
                    if (sensor_id == 0) {
                        divider = BARO_BLE_GetDivider();
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 1) {
                        divider = HUM_BLE_GetDivider();
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 2) {
                        divider = ACCEL_BLE_GetDivider();
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 3) {
                        divider = GYRO_BLE_GetDivider();
                        status = CP_STATUS_SUCCESS;
                    } else if (sensor_id == 4) {
                        divider = MAG_BLE_GetDivider();
                        status = CP_STATUS_SUCCESS;
                    } else {
                        status = CP_STATUS_INVALID_PARAMETER; // Invalid sensor_id
                    }
                    
                    if (status == CP_STATUS_SUCCESS) {
                        response_data_buf[0] = sensor_id;
                        response_data_buf[1] = divider & 0xFF;        // Low byte
                        response_data_buf[2] = (divider >> 8) & 0xFF; // High byte
                        response_data_len = 3;
                    }
                }
                break;

            default:
                status = CP_STATUS_CMD_NOT_SUPPORTED;
                break;
        }
    }

    if (notification_enabled_flag)
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
