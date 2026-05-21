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
 * Magnetometer Hard Iron Calibration
 *
 * Standalone module wrapping ST MotionFX MagCal to continuously estimate
 * and apply magnetometer hard iron bias. Designed to be independent of the
 * AHRS algorithm underneath so the AHRS can be swapped later.
 *
 * Module state persists across active/sleep mode transitions (static RAM).
 * On each active mode entry, MagCal_Init() re-applies any previously learned
 * calibration to fusion immediately, then continues refining.
 *
 * Input:  raw sensor-frame mag samples (int16_t, units: gauss * 1000)
 * Output: hard iron bias pushed to FS_Fusion_SetMagHardIron() on quality upgrade
 */

#ifndef MAG_CAL_H_
#define MAG_CAL_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MAG_CAL_QUALITY_UNKNOWN = 0,
    MAG_CAL_QUALITY_POOR,
    MAG_CAL_QUALITY_OK,
    MAG_CAL_QUALITY_GOOD
} MagCal_Quality_t;

/**
 * @brief  Initialize/restart magnetometer calibration.
 *
 * Call at the start of each active mode session (from FS_ActiveControl_Init).
 * If a valid calibration (OK or GOOD quality) already exists from a prior
 * session, it is immediately re-applied to fusion. The algorithm is always
 * (re)started so it can continue to refine the estimate.
 */
void MagCal_Init(void);

/**
 * @brief  Feed a raw magnetometer sample into the calibration algorithm.
 *
 * Call once per mag sample from FS_Mag_DataReady_Callback, passing the
 * raw sensor-frame values before axis remapping. Internally subsamples
 * to ~25 Hz. Automatically calls FS_Fusion_SetMagHardIron() whenever
 * calibration quality improves.
 *
 * @param  x  Raw X axis (gauss * 1000, sensor frame)
 * @param  y  Raw Y axis (gauss * 1000, sensor frame)
 * @param  z  Raw Z axis (gauss * 1000, sensor frame)
 */
void MagCal_Update(int16_t x, int16_t y, int16_t z);

/**
 * @brief  Return the current calibration quality level.
 */
MagCal_Quality_t MagCal_GetQuality(void);

/**
 * @brief  Return the current best hard iron estimate in gauss.
 *
 * @param  hi_gauss  Output array [3], filled with X/Y/Z hard iron in gauss.
 * @return true if a valid calibration (OK or better) exists, false otherwise.
 */
bool MagCal_GetHardIron(float hi_gauss[3]);

#endif /* MAG_CAL_H_ */
