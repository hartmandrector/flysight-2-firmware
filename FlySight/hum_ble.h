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

#ifndef HUM_BLE_H_
#define HUM_BLE_H_

#include <stdint.h>

#include "stm32wbxx_hal.h"
#include "hum.h"
#include "config.h"

/* ------------------------------------------------------------------ */
/* Packet layout control                                              */
/* ------------------------------------------------------------------ */

#define HUM_BLE_MAX_LEN            12u

/* Bit-layout of mask byte (MSB first) */
#define HUM_BLE_BIT_TIME           0x80u
#define HUM_BLE_BIT_HUMIDITY       0x40u
#define HUM_BLE_BIT_TEMPERATURE    0x20u

/* Default mask: all fields enabled */
#define HUM_BLE_DEFAULT_MASK       0xE0u

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

void     HUM_BLE_Init(const FS_Config_Data_t *config);
uint8_t  HUM_BLE_GetMask(void);
void     HUM_BLE_SetMask(uint8_t mask);
uint16_t HUM_BLE_GetDivider(void);
void     HUM_BLE_SetDivider(uint16_t divider);
uint8_t  HUM_BLE_Build(const FS_Hum_Data_t *src, uint8_t *dst);

#endif /* HUM_BLE_H_ */
