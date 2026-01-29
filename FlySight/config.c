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
#include "app_common.h"
#include "config.h"
#include "ff.h"
#include "state.h"

#define CONFIG_FIRST_ALARM   0x01
#define CONFIG_FIRST_WINDOW  0x02
#define CONFIG_FIRST_SPEECH  0x04
#define CONFIG_FIRST_AL_LINE 0x08

static FS_Config_Data_t config;
static FIL configFile;

static const char defaultConfig[] =
		"; FlySight - http://flysight.ca\n"
		"\n"
		"; GPS settings\n"
		"\n"
		"Model:     7     ; Dynamic model\n"
		"                 ;   0 = Portable\n"
		"                 ;   2 = Stationary\n"
		"                 ;   3 = Pedestrian\n"
		"                 ;   4 = Automotive\n"
		"                 ;   5 = Sea\n"
		"                 ;   6 = Airborne with < 1 G acceleration\n"
		"                 ;   7 = Airborne with < 2 G acceleration\n"
		"                 ;   8 = Airborne with < 4 G acceleration\n"
		"Rate:      200   ; Measurement rate (ms)\n"
		"\n"
		"; Alarm settings\n"
		"\n"
		"; WARNING: GPS measurements depend on very weak signals\n"
		";          received from orbiting satellites. As such, they\n"
		";          are prone to interference, and should NEVER be\n"
		";          relied upon for life saving purposes.\n"
		"\n"
		";          UNDER NO CIRCUMSTANCES SHOULD THESE ALARMS BE\n"
		";          USED TO INDICATE DEPLOYMENT OR BREAKOFF ALTITUDE.\n"
		"\n"
		"; NOTE:    Alarm elevations are given in meters above ground\n"
		";          elevation, which is specified in DZ_Elev.\n"
		"\n"
		"Win_Above:     0 ; Window above each alarm (m)\n"
		"Win_Below:     0 ; Window below each alarm (m)\n"
		"DZ_Elev:       0 ; Ground elevation (m above sea level)\n"
		"\n"
		"Alarm_Elev:    0 ; Alarm elevation (m above ground level)\n"
		"Alarm_Type:    0 ; Alarm type\n"
		"                 ;   0 = No alarm\n"
		"                 ;   1 = Beep\n"
		"                 ;   2 = Chirp up\n"
		"                 ;   3 = Chirp down\n"
		"                 ;   4 = Play file\n"
		"Alarm_File:    0 ; File to be played\n"
		";\n"
		"; Sensor Configuration\n"
		";\n"
		"; Output Data Rate (ODR) settings control the rate at which sensor\n"
		"; hardware sends data to the microcontroller. These rates determine\n"
		"; the data rate in SENSOR.CSV log files and Bluetooth streaming.\n"
		";\n"
		"; Each sensor can be individually enabled (1) or disabled (0).\n"
		"; Disabling a sensor reduces power consumption and turns off both\n"
		"; logging and Bluetooth transmission for that sensor.\n"
		";\n"
		"; GPS Rate (milliseconds):\n"
		";   1000 = 1 Hz (default)\n"
		";   500  = 2 Hz\n"
		";   200  = 5 Hz\n"
		";   100  = 10 Hz\n"
		";   50   = 20 Hz (tested, accurate)\n"
		";   40   = 25 Hz (tested, accurate, maximum recommended)\n"
		";\n"
		"; Barometer (LPS22HH) - Baro_ODR:\n"
		";   0 = Power-down / One-shot\n"
		";   1 = 1 Hz\n"
		";   2 = 10 Hz (default)\n"
		";   3 = 25 Hz\n"
		";   4 = 50 Hz\n"
		";   5 = 75 Hz\n"
		";   6 = 100 Hz (maximum)\n"
		";   7 = 200 Hz (low-noise mode, high power)\n"
		";\n"
		"Enable_Baro:   1 ; Enable barometer (0 = off, 1 = on)\n"
		"Baro_ODR:      2 ; 10 Hz\n"
		"\n"
		"; Humidity (HTS221/SHT4x) - Hum_ODR:\n"
		";   0 = Power-down / One-shot\n"
		";   1 = 1 Hz (default)\n"
		";   2 = 7 Hz (HTS221 only)\n"
		";   3 = 12.5 Hz (HTS221 maximum)\n"
		";\n"
		"Enable_Hum:    1 ; Enable humidity sensor (0 = off, 1 = on)\n"
		"Hum_ODR:       1 ; 1 Hz\n"
		"\n"
		"; Magnetometer (LIS2MDL) - Mag_ODR:\n"
		";   0 = 10 Hz (default)\n"
		";   1 = 20 Hz\n"
		";   2 = 50 Hz\n"
		";   3 = 100 Hz (maximum)\n"
		";\n"
		"Enable_Mag:    1 ; Enable magnetometer (0 = off, 1 = on)\n"
		"Mag_ODR:       0 ; 10 Hz\n"
		"\n"
		"; Accelerometer (LSM6DSO) - Accel_ODR:\n"
		";   0  = Power-down\n"
		";   1  = 12.5 Hz (default, low-power)\n"
		";   2  = 26 Hz (low-power)\n"
		";   3  = 52 Hz (low-power)\n"
		";   4  = 104 Hz (normal)\n"
		";   5  = 208 Hz (normal)\n"
		";   6  = 416 Hz (high-performance)\n"
		";   7  = 833 Hz (high-performance)\n"
		";   8  = 1666 Hz (high-performance)\n"
		";   9  = 3333 Hz (high-performance)\n"
		";   10 = 6666 Hz (high-performance, maximum)\n"
		";   11 = 1.6 Hz (ultra-low-power)\n"
		";\n"
		"Enable_IMU:    1 ; Enable IMU (accel + gyro) (0 = off, 1 = on)\n"
		"Accel_ODR:     1 ; 12.5 Hz\n"
		"Accel_FS:      0 ; Accelerometer full scale\n"
		"                 ;   0 = ±2 g\n"
		"                 ;   1 = ±4 g\n"
		"                 ;   2 = ±8 g\n"
		"                 ;   3 = ±16 g\n"
		"\n"
		"; Gyroscope (LSM6DSO) - Gyro_ODR:\n"
		";   0  = Power-down\n"
		";   1  = 12.5 Hz (default, low-power)\n"
		";   2  = 26 Hz (low-power)\n"
		";   3  = 52 Hz (low-power)\n"
		";   4  = 104 Hz (normal)\n"
		";   5  = 208 Hz (normal)\n"
		";   6  = 416 Hz (high-performance)\n"
		";   7  = 833 Hz (high-performance)\n"
		";   8  = 1666 Hz (high-performance)\n"
		";   9  = 3333 Hz (high-performance)\n"
		";   10 = 6666 Hz (high-performance, maximum)\n"
		";\n"
		"; Note: Gyro_ODR values 0-10 only. ODR value 11 (1.6 Hz)\n"
		";       is not supported for gyroscope.\n"
		";\n"
		"Gyro_ODR:      1 ; 12.5 Hz\n"
		"Gyro_FS:       0 ; Gyroscope full scale\n"
		"                 ;   0 = ±250 dps\n"
		"                 ;   1 = ±500 dps\n"
		"                 ;   2 = ±1000 dps\n"
		"                 ;   3 = ±2000 dps\n"
		"\n"
		"; Bluetooth BLE Divider Settings\n"
		";\n"
		"; These dividers control the rate at which sensor data is transmitted\n"
		"; over Bluetooth Low Energy. The BLE transmission rate equals:\n"
		";\n"
		";   BLE_Rate = Hardware_Rate / Divider\n"
		";\n"
		"; Setting to 0 enables auto-calculation (recommended), which calculates\n"
		"; optimal dividers based on sensor ODR settings and GPS rate. The firmware\n"
		"; accounts for GPS bandwidth first (~220 bytes/sec at 5Hz), then distributes\n"
		"; remaining bandwidth (~1280 bytes/sec) among enabled sensors to stay within\n"
		"; the safe BLE throughput limit of 1500 bytes/sec.\n"
		";\n"
		"; GPS transmission rate is controlled by the 'Rate' parameter above and\n"
		"; does not have a separate BLE divider. High GPS rates (e.g., 40ms = 25Hz)\n"
		"; will cause the firmware to automatically increase sensor dividers to\n"
		"; compensate for the increased GPS bandwidth.\n"
		";\n"
		"; Examples:\n"
		";   Accel_ODR: 6 (416 Hz), BLE_Accel_Divider: 0 → Auto (28) → 14.9 Hz BLE\n"
		";   Accel_ODR: 1 (12.5 Hz), BLE_Accel_Divider: 0 → Auto (1) → 12.5 Hz BLE\n"
		";   Baro_ODR: 2 (10 Hz), BLE_Baro_Divider: 1 → 10 Hz BLE (no divider)\n"
		";\n"
		"; WARNING: Setting manual dividers too low may exceed BLE bandwidth limits,\n"
		";          causing buffer overrun and connection loss. Auto (0) is strongly\n"
		";          recommended for reliable operation.\n"
		";\n"
		"BLE_Baro_Divider:  0 ; Auto-calculate (recommended)\n"
		"BLE_Hum_Divider:   0 ; Auto-calculate (recommended)\n"
		"BLE_Accel_Divider: 0 ; Auto-calculate (recommended)\n"
		"BLE_Gyro_Divider:  0 ; Auto-calculate (recommended)\n"
		"BLE_Mag_Divider:   0 ; Auto-calculate (recommended)\n";

void FS_Config_Init(void)
{
	config.model         = FS_CONFIG_MODEL_AIRBORNE_2G;
	config.rate          = 200;

	config.mode          = FS_CONFIG_MODE_GLIDE_RATIO;
	config.min           = 0;
	config.max           = 300;
	config.limits        = 1;
	config.volume        = 2;

	config.mode_2        = FS_CONFIG_MODE_CHANGE_IN_VALUE_1;
	config.min_2         = 300;
	config.max_2         = 1500;
	config.min_rate      = FS_CONFIG_RATE_ONE_HZ;
	config.max_rate      = 5 * FS_CONFIG_RATE_ONE_HZ;
	config.flatline      = 0;

	config.sp_rate       = 0;
	config.sp_volume     = 0;
	config.num_speech    = 0;

	config.threshold     = 1000;
	config.hThreshold    = 0;

	config.use_sas       = 1;
	config.tz_offset     = 0;

	config.init_mode     = 0;
	*(config.init_filename) = '\0';

	config.alarm_window_above = 0;
	config.alarm_window_below = 0;
	config.dz_elev       = 0;

	config.num_alarms    = 0;

	config.alt_units     = FS_CONFIG_UNITS_FEET;
	config.alt_step      = 0;

	config.num_windows   = 0;

	config.enable_audio   = 1;
	config.enable_logging = 1;
	config.enable_vbat    = 1;
	config.enable_mic     = 1;
	config.enable_imu     = 1;
	config.enable_gnss    = 1;
	config.enable_baro    = 1;
	config.enable_hum     = 1;
	config.enable_mag     = 1;
	config.ble_tx_power   = 25;
	config.enable_raw     = 1;
	config.cold_start     = 0;

	config.baro_odr       = 2;
	config.hum_odr        = 1;
	config.mag_odr        = 0;
	config.accel_odr      = 1;
	config.accel_fs       = 1;
	config.gyro_odr       = 1;
	config.gyro_fs        = 3;

	config.ble_baro_divider  = 0;  // Auto-calculate
	config.ble_hum_divider   = 0;  // Auto-calculate
	config.ble_accel_divider = 0;  // Auto-calculate
	config.ble_gyro_divider  = 0;  // Auto-calculate
	config.ble_mag_divider   = 0;  // Auto-calculate

	config.lat            = 0;
	config.lon            = 0;
	config.bearing        = 0;
	config.end_nav        = 0;
	config.max_dist       = 10000;
	config.min_angle      = 5;

	*(config.al_id) = '\0';
	config.al_mode        = 1;
	config.al_rate        = 1000;
	config.num_al_lines   = 0;

	// IMPORTANT: Navigation disabled by default
	config.enable_nav     = 0;
}

static void FS_Config_WriteHex_32(char *result, const uint32_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		sprintf(result + 8 * i, "%08lx", data[i]);
	}
}

FS_Config_Result_t FS_Config_Read(const char *filename)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	int32_t val;

	uint8_t flags = 0;

	if (f_open(&configFile, filename, FA_READ) != FR_OK)
		return FS_CONFIG_ERR;

	while (!f_eof(&configFile))
	{
		f_gets(buffer, sizeof(buffer), &configFile);

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		name = strtok(buffer, " \r\n\t:");
		if (name == 0) continue ;

		result = strtok(0, " \r\n\t:");
		if (result == 0) continue ;

		val = atol(result);

		#define HANDLE_VALUE(s,w,r,t) \
			if ((t) && !strcmp(name, (s))) { (w) = (r); }

		HANDLE_VALUE("Model",     config.model,        val, val >= 0 && val <= 8);
		HANDLE_VALUE("Rate",      config.rate,         val, val >= 40 && val <= 1000);
		HANDLE_VALUE("Mode",      config.mode,         val, (val >= 0 && val <= 7) || (val == 11));
		HANDLE_VALUE("Min",       config.min,          val, TRUE);
		HANDLE_VALUE("Max",       config.max,          val, TRUE);
		HANDLE_VALUE("Limits",    config.limits,       val, val >= 0 && val <= 3);
		HANDLE_VALUE("Volume",    config.volume,       8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("Mode_2",    config.mode_2,       val, (val >= 0 && val <= 9) || (val == 11));
		HANDLE_VALUE("Min_Val_2", config.min_2,        val, TRUE);
		HANDLE_VALUE("Max_Val_2", config.max_2,        val, TRUE);
		HANDLE_VALUE("Min_Rate",  config.min_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Max_Rate",  config.max_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Flatline",  config.flatline,     val, val == 0 || val == 1);
		HANDLE_VALUE("Sp_Rate",   config.sp_rate,      val * 1000, val >= 0 && val <= 32);
		HANDLE_VALUE("Sp_Volume", config.sp_volume,    8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("V_Thresh",  config.threshold,    val, TRUE);
		HANDLE_VALUE("H_Thresh",  config.hThreshold,   val, TRUE);
		HANDLE_VALUE("Use_SAS",   config.use_sas,      val, val == 0 || val == 1);
		HANDLE_VALUE("Window",    config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Window",    config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("Win_Above", config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Win_Below", config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("DZ_Elev",   config.dz_elev,      val * 1000, TRUE);
		HANDLE_VALUE("TZ_Offset", config.tz_offset,    val, TRUE);
		HANDLE_VALUE("Init_Mode", config.init_mode,    val, val >= 0 && val <= 2);
		HANDLE_VALUE("Alt_Units", config.alt_units,    val, val >= 0 && val <= 1);
		HANDLE_VALUE("Alt_Step",  config.alt_step,     val, val >= 0);

		HANDLE_VALUE("Enable_Audio",   config.enable_audio,   val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Logging", config.enable_logging, val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Vbat",    config.enable_vbat,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mic",     config.enable_mic,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Imu",     config.enable_imu,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Gnss",    config.enable_gnss,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Baro",    config.enable_baro,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Hum",     config.enable_hum,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mag",     config.enable_mag,     val, val == 0 || val == 1);
		HANDLE_VALUE("Ble_Tx_Power",   config.ble_tx_power,   val, val >= 0 && val <= 31);
		HANDLE_VALUE("Enable_Raw",     config.enable_raw,     val, val == 0 || val == 1);
		HANDLE_VALUE("Cold_Start",     config.cold_start,     val, val == 0 || val == 1);

		HANDLE_VALUE("Baro_ODR",  config.baro_odr,     val, val >= 0 && val <= 7);
		HANDLE_VALUE("Hum_ODR",   config.hum_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Mag_ODR",   config.mag_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Accel_ODR", config.accel_odr,    val, val >= 0 && val <= 11);
		HANDLE_VALUE("Accel_FS",  config.accel_fs,     val, val >= 0 && val <= 3);
		HANDLE_VALUE("Gyro_ODR",  config.gyro_odr,     val, val >= 0 && val <= 10);
		HANDLE_VALUE("Gyro_FS",   config.gyro_fs,      val, val >= 0 && val <= 3);

		HANDLE_VALUE("BLE_Baro_Divider",  config.ble_baro_divider,  val, val >= 0 && val <= 65535);
		HANDLE_VALUE("BLE_Hum_Divider",   config.ble_hum_divider,   val, val >= 0 && val <= 65535);
		HANDLE_VALUE("BLE_Accel_Divider", config.ble_accel_divider, val, val >= 0 && val <= 65535);
		HANDLE_VALUE("BLE_Gyro_Divider",  config.ble_gyro_divider,  val, val >= 0 && val <= 65535);
		HANDLE_VALUE("BLE_Mag_Divider",   config.ble_mag_divider,   val, val >= 0 && val <= 65535);

		HANDLE_VALUE("Lat",       config.lat,          val, val >= -900000000 && val <= 900000000);
		HANDLE_VALUE("Lon",       config.lon,          val, val >= -1800000000 && val <= 1800000000);
		HANDLE_VALUE("Bearing",   config.bearing,      val, val >= 0 && val <= 360);
		HANDLE_VALUE("End_Nav",   config.end_nav,      val * 1000, TRUE);
		HANDLE_VALUE("Max_Dist",  config.max_dist,     val, val >= 0 && val <= 10000);
		HANDLE_VALUE("Min_Angle", config.min_angle,    val, val >= 0 && val <= 360);

		HANDLE_VALUE("AL_Mode",   config.al_mode,      val, val >= 0 && val <= 1);
		HANDLE_VALUE("AL_Rate",   config.al_rate,      val, val >= 100);

		#undef HANDLE_VALUE

		if (!strcmp(name, "Init_File"))
		{
			result[8] = '\0';
			strncpy(config.init_filename, result, sizeof(config.init_filename));
		}

		if (!strcmp(name, "Alarm_Elev") && config.num_alarms < FS_CONFIG_MAX_ALARMS)
		{
			if (!(flags & CONFIG_FIRST_ALARM))
			{
				config.num_alarms = 0;
				flags |= CONFIG_FIRST_ALARM;
			}

			++config.num_alarms;
			config.alarms[config.num_alarms - 1].elev = val * 1000;
			config.alarms[config.num_alarms - 1].type = 0;
			config.alarms[config.num_alarms - 1].filename[0] = '\0';
		}
		if (!strcmp(name, "Alarm_Type") && config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			config.alarms[config.num_alarms - 1].type = val;
		}
		if (!strcmp(name, "Alarm_File") && config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			result[8] = '\0';
			strncpy(config.alarms[config.num_alarms - 1].filename, result,
					sizeof(config.alarms[config.num_alarms - 1].filename));
		}

		if (!strcmp(name, "Win_Top") && config.num_windows < FS_CONFIG_MAX_WINDOWS)
		{
			if (!(flags & CONFIG_FIRST_WINDOW))
			{
				config.num_windows = 0;
				flags |= CONFIG_FIRST_WINDOW;
			}

			++config.num_windows;
			config.windows[config.num_windows - 1].top = val * 1000;
		}
		if (!strcmp(name, "Win_Bottom") && config.num_windows <= FS_CONFIG_MAX_WINDOWS)
		{
			config.windows[config.num_windows - 1].bottom = val * 1000;
		}

		if (!strcmp(name, "Sp_Mode") && config.num_speech < FS_CONFIG_MAX_SPEECH)
		{
			if (!(flags & CONFIG_FIRST_SPEECH))
			{
				config.num_speech = 0;
				flags |= CONFIG_FIRST_SPEECH;
			}

			++config.num_speech;
			config.speech[config.num_speech - 1].mode = val;
			config.speech[config.num_speech - 1].units = FS_CONFIG_UNITS_MPH;
			config.speech[config.num_speech - 1].decimals = 1;
		}
		if (!strcmp(name, "Sp_Units") && config.num_speech <= FS_CONFIG_MAX_SPEECH)
		{
			config.speech[config.num_speech - 1].units = val;
		}
		if (!strcmp(name, "Sp_Dec") && config.num_speech <= FS_CONFIG_MAX_SPEECH)
		{
			config.speech[config.num_speech - 1].decimals = val;
		}

		if (!strcmp(name, "AL_Line") && config.num_al_lines < FS_CONFIG_MAX_AL_LINES)
		{
			if (!(flags & CONFIG_FIRST_AL_LINE))
			{
				config.num_al_lines = 0;
				flags |= CONFIG_FIRST_AL_LINE;
			}

			++config.num_al_lines;
			config.al_lines[config.num_al_lines - 1].mode = val;
			config.al_lines[config.num_al_lines - 1].units = FS_UNIT_SYSTEM_IMPERIAL;
			config.al_lines[config.num_al_lines - 1].decimals = 1;
		}
		if (!strcmp(name, "AL_Units") && config.num_al_lines <= FS_CONFIG_MAX_AL_LINES)
		{
			config.al_lines[config.num_al_lines - 1].units = val;
		}
		if (!strcmp(name, "AL_Dec") && config.num_al_lines <= FS_CONFIG_MAX_AL_LINES)
		{
			config.al_lines[config.num_al_lines - 1].decimals = val;
		}

		if (!strcmp(name, "AL_ID"))
		{
			result[6] = '\0';
			strncpy(config.al_id, result, sizeof(config.al_id));
		}

		// Compare config Device_ID with actual hardware device ID.
		// If it matches, enable navigation.
		if (!strcmp(name, "Device_ID"))
		{
			// Build 24-hex-digit string from the hardware ID
			const uint32_t *hwId = FS_State_Get()->device_id; // 3 x 32-bit
			char hwIdString[25];  // 24 hex digits + null
			FS_Config_WriteHex_32(hwIdString, hwId, 3);

			if (!strcmp(result, hwIdString))
			{
				config.enable_nav = 1;
			}
		}
	}

	f_close(&configFile);

	// Auto-calculate BLE dividers if set to 0 (taking GPS bandwidth into account)
	FS_BLE_AutoCalculateDividers(&config);

	return FS_CONFIG_OK;
}

FS_Config_Result_t FS_Config_Write(const char *filename)
{
	FRESULT res;

	res = f_open(&configFile, filename, FA_WRITE|FA_CREATE_ALWAYS);
	if (res != FR_OK) return FS_CONFIG_ERR;

	f_puts(defaultConfig, &configFile);

	f_close(&configFile);

	return FS_CONFIG_OK;
}

const FS_Config_Data_t *FS_Config_Get(void)
{
	return &config;
}
