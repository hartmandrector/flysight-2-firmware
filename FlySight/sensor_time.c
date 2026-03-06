/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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

#include "main.h"
#include "sensor_time.h"

static volatile uint32_t overflow_count;

void FS_SensorTime_Init(void)
{
	/* Enable TIM2 clock */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* Configure TIM2 as free-running 1 MHz counter */
	TIM2->PSC = 31;          /* 32 MHz / 32 = 1 MHz = 1 us per tick */
	TIM2->ARR = 0xFFFFFFFF;  /* Full 32-bit range */
	TIM2->CNT = 0;
	TIM2->CR1 = 0;           /* Upcounting, no auto-reload preload */

	/* Generate update event to load prescaler, then clear the flag */
	TIM2->EGR = TIM_EGR_UG;
	TIM2->SR = ~TIM_SR_UIF;

	/* Enable update interrupt */
	TIM2->DIER = TIM_DIER_UIE;

	/* Configure NVIC for TIM2 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void FS_SensorTime_Start(void)
{
	/* Reset counter and overflow */
	overflow_count = 0;
	TIM2->CNT = 0;
	TIM2->SR = ~TIM_SR_UIF;

	/* Start counting */
	TIM2->CR1 |= TIM_CR1_CEN;
}

void FS_SensorTime_Stop(void)
{
	/* Stop counting */
	TIM2->CR1 &= ~TIM_CR1_CEN;

	/* Disable TIM2 clock to save power */
	__HAL_RCC_TIM2_CLK_DISABLE();
}

uint64_t FS_SensorTime_GetTicks(void)
{
	uint32_t hi1, hi2, lo;

	do {
		hi1 = overflow_count;
		lo = TIM2->CNT;
		hi2 = overflow_count;
	} while (hi1 != hi2);

	return ((uint64_t)hi1 << 32) | lo;
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR = ~TIM_SR_UIF;
		++overflow_count;
	}
}
