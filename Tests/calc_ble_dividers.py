#!/usr/bin/env python3
"""
BLE Divider Calculator - FlySight 2
Standalone tool for calculating BLE divider values and bandwidth usage.
Implements the same priority-based algorithm as ble_config.c
"""

import sys
import argparse
import math

# Constants
BLE_BANDWIDTH_LIMIT = 1500  # bytes/sec
BLE_MIN_RATE_HZ = 0.5       # Minimum 0.5 Hz per sensor

# Packet sizes
GPS_PV_PACKET_SIZE = 44
BARO_PACKET_SIZE = 11
HUM_PACKET_SIZE = 9
ACCEL_PACKET_SIZE = 19
GYRO_PACKET_SIZE = 27  # Includes quaternion (default mask 0xF0)
MAG_PACKET_SIZE = 13

# ODR mappings
BARO_ODR_MAP = {0: 10.0, 1: 20.0, 2: 50.0, 3: 100.0}
HUM_ODR_MAP = {0: 1.0}  # Fixed 1 Hz
IMU_ODR_MAP = {0: 1.6, 1: 12.5, 2: 26.0, 3: 52.0, 4: 104.0, 5: 208.0, 6: 416.0}

def odr_to_hz(odr_idx, sensor_type):
    """Convert ODR index to Hz for given sensor type"""
    if sensor_type == 'baro':
        return BARO_ODR_MAP.get(odr_idx, 10.0)
    elif sensor_type == 'hum':
        return HUM_ODR_MAP.get(odr_idx, 1.0)
    else:  # accel, gyro, mag
        return IMU_ODR_MAP.get(odr_idx, 12.5)

def calculate_max_divider(hw_rate_hz):
    """Calculate maximum divider to maintain minimum rate (0.5 Hz)"""
    return int(hw_rate_hz * 2)  # hw_rate / divider >= 0.5, so divider <= hw_rate * 2

def auto_calculate_dividers(config):
    """
    Priority-based BLE divider calculation algorithm.
    Modifies config dict in-place.
    Returns dict with auto-calculated dividers.
    """
    # Step 1: Calculate GPS bandwidth (always present)
    gps_rate_hz = 1000.0 / config['gnss_rate']
    gps_bandwidth = gps_rate_hz * GPS_PV_PACKET_SIZE
    
    # Step 2: Calculate manual divider bandwidth
    manual_bandwidth = 0.0
    
    sensors = [
        ('baro', config['baro_odr'], config['ble_baro_divider'], BARO_PACKET_SIZE),
        ('hum', config['hum_odr'], config['ble_hum_divider'], HUM_PACKET_SIZE),
        ('accel', config['accel_odr'], config['ble_accel_divider'], ACCEL_PACKET_SIZE),
        ('gyro', config['gyro_odr'], config['ble_gyro_divider'], GYRO_PACKET_SIZE),
        ('mag', config['mag_odr'], config['ble_mag_divider'], MAG_PACKET_SIZE),
    ]
    
    for sensor_type, odr_idx, divider, packet_size in sensors:
        if divider > 0:  # Manual divider
            hw_rate = odr_to_hz(odr_idx, sensor_type)
            ble_rate = hw_rate / divider
            manual_bandwidth += ble_rate * packet_size
    
    # Step 3: Calculate remaining budget
    remaining_budget = BLE_BANDWIDTH_LIMIT - gps_bandwidth - manual_bandwidth
    
    # Step 4: Try auto sensors at full ODR (divider=1)
    auto_bandwidth = 0.0
    auto_sensors = []
    
    for sensor_type, odr_idx, divider, packet_size in sensors:
        if divider == 0:  # Auto mode
            hw_rate = odr_to_hz(odr_idx, sensor_type)
            ble_rate = hw_rate  # divider=1
            auto_bandwidth += ble_rate * packet_size
            auto_sensors.append((sensor_type, odr_idx, hw_rate, packet_size))
    
    # Step 5: Check if scaling needed
    if auto_bandwidth <= remaining_budget:
        # No scaling needed - use divider=1 for all auto sensors
        for sensor_type, _, _, _ in auto_sensors:
            if sensor_type == 'baro':
                config['ble_baro_divider'] = 1
            elif sensor_type == 'hum':
                config['ble_hum_divider'] = 1
            elif sensor_type == 'accel':
                config['ble_accel_divider'] = 1
            elif sensor_type == 'gyro':
                config['ble_gyro_divider'] = 1
            elif sensor_type == 'mag':
                config['ble_mag_divider'] = 1
    else:
        # Scaling required - calculate scale factor
        scale_factor = auto_bandwidth / remaining_budget
        
        # Apply scaled dividers to all auto sensors
        for sensor_type, odr_idx, hw_rate, _ in auto_sensors:
            scaled_divider = math.ceil(scale_factor)
            
            # Enforce minimum rate (0.5 Hz)
            max_divider = calculate_max_divider(hw_rate)
            if scaled_divider > max_divider:
                scaled_divider = max_divider
            
            if sensor_type == 'baro':
                config['ble_baro_divider'] = scaled_divider
            elif sensor_type == 'hum':
                config['ble_hum_divider'] = scaled_divider
            elif sensor_type == 'accel':
                config['ble_accel_divider'] = scaled_divider
            elif sensor_type == 'gyro':
                config['ble_gyro_divider'] = scaled_divider
            elif sensor_type == 'mag':
                config['ble_mag_divider'] = scaled_divider

def calculate_bandwidth(config):
    """Calculate total bandwidth and per-sensor breakdown"""
    gps_rate = 1000.0 / config['gnss_rate']
    gps_bw = gps_rate * GPS_PV_PACKET_SIZE
    
    # Treat divider=0 as divider=1 for display (auto mode means it was calculated)
    baro_div = config['ble_baro_divider'] if config['ble_baro_divider'] > 0 else 1
    hum_div = config['ble_hum_divider'] if config['ble_hum_divider'] > 0 else 1
    accel_div = config['ble_accel_divider'] if config['ble_accel_divider'] > 0 else 1
    gyro_div = config['ble_gyro_divider'] if config['ble_gyro_divider'] > 0 else 1
    mag_div = config['ble_mag_divider'] if config['ble_mag_divider'] > 0 else 1
    
    baro_rate = odr_to_hz(config['baro_odr'], 'baro') / baro_div
    baro_bw = baro_rate * BARO_PACKET_SIZE
    
    hum_rate = odr_to_hz(config['hum_odr'], 'hum') / hum_div
    hum_bw = hum_rate * HUM_PACKET_SIZE
    
    accel_rate = odr_to_hz(config['accel_odr'], 'accel') / accel_div
    accel_bw = accel_rate * ACCEL_PACKET_SIZE
    
    gyro_rate = odr_to_hz(config['gyro_odr'], 'gyro') / gyro_div
    gyro_bw = gyro_rate * GYRO_PACKET_SIZE
    
    mag_rate = odr_to_hz(config['mag_odr'], 'mag') / mag_div
    mag_bw = mag_rate * MAG_PACKET_SIZE
    
    total = gps_bw + baro_bw + hum_bw + accel_bw + gyro_bw + mag_bw
    
    return {
        'gps': (gps_rate, gps_bw),
        'baro': (baro_rate, baro_bw),
        'hum': (hum_rate, hum_bw),
        'accel': (accel_rate, accel_bw),
        'gyro': (gyro_rate, gyro_bw),
        'mag': (mag_rate, mag_bw),
        'total': total
    }

def validate_config(bandwidth_info):
    """Validate if configuration is within bandwidth limit"""
    return bandwidth_info['total'] <= BLE_BANDWIDTH_LIMIT

def print_config(config, title):
    """Print configuration section"""
    print("=" * 63)
    print(f"{title:^63}")
    print("=" * 63)
    
    gps_rate = 1000.0 / config['gnss_rate']
    print(f"GPS Rate:      {config['gnss_rate']} ms ({gps_rate:.2f} Hz)")
    
    print("\nSensor ODRs:")
    print(f"  Baro:        {odr_to_hz(config['baro_odr'], 'baro'):.1f} Hz (ODR={config['baro_odr']})")
    print(f"  Humidity:    {odr_to_hz(config['hum_odr'], 'hum'):.1f} Hz (ODR={config['hum_odr']})")
    print(f"  Accel:       {odr_to_hz(config['accel_odr'], 'accel'):.1f} Hz (ODR={config['accel_odr']})")
    print(f"  Gyro:        {odr_to_hz(config['gyro_odr'], 'gyro'):.1f} Hz (ODR={config['gyro_odr']})")
    print(f"  Mag:         {odr_to_hz(config['mag_odr'], 'mag'):.1f} Hz (ODR={config['mag_odr']})")

def print_dividers(config, input_dividers):
    """Print input and calculated dividers"""
    print("\nInput Dividers:")
    for name, key in [('Baro', 'ble_baro_divider'), ('Humidity', 'ble_hum_divider'), 
                       ('Accel', 'ble_accel_divider'), ('Gyro', 'ble_gyro_divider'), 
                       ('Mag', 'ble_mag_divider')]:
        div = input_dividers[key]
        mode = "(AUTO)" if div == 0 else "(MANUAL)"
        print(f"  {name:12} {div} {mode}")
    
    print("\n" + "=" * 63)
    print(f"{'CALCULATED DIVIDERS':^63}")
    print("=" * 63)
    print(f"  Baro:        {config['ble_baro_divider']}")
    print(f"  Humidity:    {config['ble_hum_divider']}")
    print(f"  Accel:       {config['ble_accel_divider']}")
    print(f"  Gyro:        {config['ble_gyro_divider']}")
    print(f"  Mag:         {config['ble_mag_divider']}")

def print_rates_and_bandwidth(bw_info):
    """Print BLE rates and bandwidth usage"""
    print("\n" + "=" * 63)
    print(f"{'BLE RATES':^63}")
    print("=" * 63)
    print(f"  GPS:         {bw_info['gps'][0]:.2f} Hz ({GPS_PV_PACKET_SIZE} bytes/packet)")
    print(f"  Baro:        {bw_info['baro'][0]:.2f} Hz ({BARO_PACKET_SIZE} bytes/packet)")
    print(f"  Humidity:    {bw_info['hum'][0]:.2f} Hz ({HUM_PACKET_SIZE} bytes/packet)")
    print(f"  Accel:       {bw_info['accel'][0]:.2f} Hz ({ACCEL_PACKET_SIZE} bytes/packet)")
    print(f"  Gyro:        {bw_info['gyro'][0]:.2f} Hz ({GYRO_PACKET_SIZE} bytes/packet)")
    print(f"  Mag:         {bw_info['mag'][0]:.2f} Hz ({MAG_PACKET_SIZE} bytes/packet)")
    
    print("\n" + "=" * 63)
    print(f"{'BANDWIDTH USAGE':^63}")
    print("=" * 63)
    for name, key in [('GPS', 'gps'), ('Baro', 'baro'), ('Humidity', 'hum'), 
                       ('Accel', 'accel'), ('Gyro', 'gyro'), ('Mag', 'mag')]:
        bw = bw_info[key][1]
        pct = (bw / BLE_BANDWIDTH_LIMIT) * 100
        print(f"  {name:12} {bw:.1f} bytes/sec ({pct:.1f}%)")
    
    print("  " + "-" * 59)
    total = bw_info['total']
    total_pct = (total / BLE_BANDWIDTH_LIMIT) * 100
    print(f"  TOTAL:       {total:.1f} bytes/sec ({total_pct:.1f}% of {BLE_BANDWIDTH_LIMIT} limit)")

def print_validation(is_valid, bw_info):
    """Print validation result"""
    print("\n" + "=" * 63)
    print(f"{'VALIDATION RESULT':^63}")
    print("=" * 63)
    if is_valid:
        print("  [OK] VALID - Configuration within bandwidth limit")
    else:
        print("  [FAIL] INVALID - Configuration exceeds bandwidth limit")
        exceeded = bw_info['total'] - BLE_BANDWIDTH_LIMIT
        exceeded_pct = (exceeded / BLE_BANDWIDTH_LIMIT) * 100
        print(f"  Exceeded by: {exceeded:.1f} bytes/sec ({exceeded_pct:.1f}%)")
    print("=" * 63)

def main():
    parser = argparse.ArgumentParser(
        description='BLE Divider Calculator - FlySight 2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default configuration (all auto)
  %(prog)s

  # High IMU rate (416 Hz accel/gyro)
  %(prog)s --accel-odr 6 --gyro-odr 6

  # Manual divider (accel at 20 Hz = 416/20)
  %(prog)s --accel-odr 6 --accel-div 20

  # High GPS rate (25 Hz)
  %(prog)s --gps-ms 40 --accel-odr 6 --gyro-odr 6
        """
    )
    
    parser.add_argument('--gps-ms', type=int, default=200, 
                        help='GPS rate in milliseconds (default: 200 = 5Hz)')
    parser.add_argument('--baro-odr', type=int, default=0, 
                        help='Baro ODR: 0=10Hz, 1=20Hz, 2=50Hz, 3=100Hz (default: 0)')
    parser.add_argument('--hum-odr', type=int, default=0, 
                        help='Hum ODR: always 1Hz (default: 0)')
    parser.add_argument('--accel-odr', type=int, default=1, 
                        help='Accel ODR: 0=1.6Hz, 1=12.5Hz, 2=26Hz, 3=52Hz, 4=104Hz, 5=208Hz, 6=416Hz (default: 1)')
    parser.add_argument('--gyro-odr', type=int, default=1, 
                        help='Gyro ODR: same as accel (default: 1)')
    parser.add_argument('--mag-odr', type=int, default=1, 
                        help='Mag ODR: same as accel (default: 1)')
    parser.add_argument('--baro-div', type=int, default=0, 
                        help='Baro divider: 0=auto, >0=manual (default: 0)')
    parser.add_argument('--hum-div', type=int, default=0, 
                        help='Hum divider: 0=auto, >0=manual (default: 0)')
    parser.add_argument('--accel-div', type=int, default=0, 
                        help='Accel divider: 0=auto, >0=manual (default: 0)')
    parser.add_argument('--gyro-div', type=int, default=0, 
                        help='Gyro divider: 0=auto, >0=manual (default: 0)')
    parser.add_argument('--mag-div', type=int, default=0, 
                        help='Mag divider: 0=auto, >0=manual (default: 0)')
    
    args = parser.parse_args()
    
    # Build config dictionary
    config = {
        'gnss_rate': args.gps_ms,
        'baro_odr': args.baro_odr,
        'hum_odr': args.hum_odr,
        'accel_odr': args.accel_odr,
        'gyro_odr': args.gyro_odr,
        'mag_odr': args.mag_odr,
        'ble_baro_divider': args.baro_div,
        'ble_hum_divider': args.hum_div,
        'ble_accel_divider': args.accel_div,
        'ble_gyro_divider': args.gyro_div,
        'ble_mag_divider': args.mag_div,
    }
    
    # Save input dividers for display
    input_dividers = {
        'ble_baro_divider': config['ble_baro_divider'],
        'ble_hum_divider': config['ble_hum_divider'],
        'ble_accel_divider': config['ble_accel_divider'],
        'ble_gyro_divider': config['ble_gyro_divider'],
        'ble_mag_divider': config['ble_mag_divider'],
    }
    
    # Print input configuration
    print_config(config, "INPUT CONFIGURATION")
    
    # Run auto-calculation
    auto_calculate_dividers(config)
    
    # Print dividers (both input and calculated)
    print_dividers(config, input_dividers)
    
    # Calculate bandwidth
    bw_info = calculate_bandwidth(config)
    
    # Print results
    print_rates_and_bandwidth(bw_info)
    
    # Validate and print result
    is_valid = validate_config(bw_info)
    print_validation(is_valid, bw_info)
    
    # Exit with appropriate code
    sys.exit(0 if is_valid else 1)

if __name__ == '__main__':
    main()


