"""
analyze_sensor.py - Analyze a FlySight 2 SENSOR.CSV file.

Reads IMU (quaternion + accel + gyro), BARO, and MAG streams and prints:
  - Run duration and sample counts
  - Filter convergence: first non-zero quaternion, time-to-lock
  - Quaternion statistics (norm stability, drift)
  - Accel/gyro ranges and noise floor (during stationary periods)
  - Mag field magnitude and hard-iron sanity check
  - Baro pressure / altitude statistics

Usage:
  python analyze_sensor.py <SENSOR.CSV>
  python analyze_sensor.py <SENSOR.CSV> --imu-trace   # print all IMU rows
  python analyze_sensor.py <SENSOR.CSV> --imu-sample N  # print every Nth IMU row

Typical workflow after a firmware test flash:
  1. python Scripts/read_magcal.py d:/MAGCAL.BIN          # check hard iron
  2. python Scripts/analyze_sensor.py d:/YY-MM-DD/HH-MM-SS/SENSOR.CSV
"""

import csv
import math
import sys
import argparse
import os


def parse_args():
    p = argparse.ArgumentParser(description="Analyze FlySight 2 SENSOR.CSV")
    p.add_argument("path", help="Path to SENSOR.CSV")
    p.add_argument("--imu-trace", action="store_true", help="Print all IMU rows")
    p.add_argument("--imu-sample", type=int, default=0, metavar="N",
                   help="Print every Nth IMU row (0=off)")
    p.add_argument("--mag-trace", action="store_true", help="Print all MAG rows")
    return p.parse_args()


def qnorm(qw, qx, qy, qz):
    return math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)


def quat_to_euler_deg(qw, qx, qy, qz):
    """Convert unit quaternion (ENU frame) to roll/pitch/yaw in degrees."""
    # Roll (x-axis rotation)
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # Yaw (z-axis rotation)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def main():
    args = parse_args()
    path = args.path

    if not os.path.exists(path):
        print(f"ERROR: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    imu_rows  = []   # (time, wx,wy,wz, ax,ay,az, temp, qw,qx,qy,qz)
    mag_rows  = []   # (time, mx,my,mz)
    baro_rows = []   # (time, pressure, temp)

    with open(path, "r", newline="", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("$"):
                continue
            parts = line.split(",")
            tag = parts[0]
            try:
                if tag == "$IMU" and len(parts) >= 13:
                    t  = float(parts[1])
                    wx = float(parts[2]); wy = float(parts[3]); wz = float(parts[4])
                    ax = float(parts[5]); ay = float(parts[6]); az = float(parts[7])
                    temp = float(parts[8])
                    qw = float(parts[9]);  qx = float(parts[10])
                    qy = float(parts[11]); qz = float(parts[12])
                    imu_rows.append((t, wx, wy, wz, ax, ay, az, temp, qw, qx, qy, qz))
                elif tag == "$MAG" and len(parts) >= 5:
                    t  = float(parts[1])
                    mx = float(parts[2]); my = float(parts[3]); mz = float(parts[4])
                    mag_rows.append((t, mx, my, mz))
                elif tag == "$BARO" and len(parts) >= 4:
                    t  = float(parts[1])
                    pr = float(parts[2]); bt = float(parts[3])
                    baro_rows.append((t, pr, bt))
            except (ValueError, IndexError):
                pass

    if not imu_rows:
        print("No IMU data found.")
        return

    n_imu = len(imu_rows)
    t_start = imu_rows[0][0]
    t_end   = imu_rows[-1][0]
    duration = t_end - t_start
    rate = (n_imu - 1) / duration if duration > 0 else 0.0

    print(f"{'='*60}")
    print(f"SENSOR.CSV: {os.path.basename(os.path.dirname(path))}/{os.path.basename(path)}")
    print(f"{'='*60}")
    print(f"\n--- Overview ---")
    print(f"  IMU samples : {n_imu}  ({rate:.1f} Hz, {duration:.2f} s)")
    print(f"  MAG samples : {len(mag_rows)}")
    print(f"  BARO samples: {len(baro_rows)}")
    print(f"  Time span   : {t_start:.3f} s -> {t_end:.3f} s")

    # --- Filter convergence ---
    # "Initialising" samples have qx=qy=qz=0 exactly (filter outputs identity).
    # Real outputs always have at least one non-zero component.
    first_real_idx = None
    for i, row in enumerate(imu_rows):
        qw, qx, qy, qz = row[8], row[9], row[10], row[11]
        if abs(qx) + abs(qy) + abs(qz) > 1e-6:  # at least one non-zero component
            first_real_idx = i
            break

    print(f"\n--- Filter Convergence ---")
    if first_real_idx is None:
        print("  WARNING: No real quaternion output found (all-zero throughout)")
        print("  This is expected for the first run after cold boot.")
        return
    elif first_real_idx == 0:
        print("  First sample already has quaternion output.")
    else:
        t_conv = imu_rows[first_real_idx][0] - t_start
        print(f"  Initialising period : samples 0-{first_real_idx-1} ({imu_rows[first_real_idx-1][0] - t_start:.3f} s)")
        print(f"  First real output   : sample {first_real_idx} at t={imu_rows[first_real_idx][0]:.3f} s  ({t_conv*1000:.0f} ms after start)")

    r0 = imu_rows[first_real_idx]
    qw0, qx0, qy0, qz0 = r0[8], r0[9], r0[10], r0[11]
    roll0, pitch0, yaw0 = quat_to_euler_deg(qw0, qx0, qy0, qz0)
    print(f"  Initial quaternion  : ({qw0:+.4f}, {qx0:+.4f}, {qy0:+.4f}, {qz0:+.4f})  |q|={qnorm(qw0,qx0,qy0,qz0):.6f}")
    print(f"  Initial Euler (ENU) : roll={roll0:+.1f}°  pitch={pitch0:+.1f}°  yaw={yaw0:+.1f}°")
    print(f"  Accel at first quat : ax={r0[4]:+.4f}g  ay={r0[5]:+.4f}g  az={r0[6]:+.4f}g")

    # Final quaternion
    rf = imu_rows[-1]
    qwf, qxf, qyf, qzf = rf[8], rf[9], rf[10], rf[11]
    rollf, pitchf, yawf = quat_to_euler_deg(qwf, qxf, qyf, qzf)
    print(f"\n--- Final Orientation (t={rf[0]:.3f} s) ---")
    print(f"  Quaternion  : ({qwf:+.4f}, {qxf:+.4f}, {qyf:+.4f}, {qzf:+.4f})  |q|={qnorm(qwf,qxf,qyf,qzf):.6f}")
    print(f"  Euler (ENU) : roll={rollf:+.1f}°  pitch={pitchf:+.1f}°  yaw={yawf:+.1f}°")
    print(f"  Accel       : ax={rf[4]:+.4f}g  ay={rf[5]:+.4f}g  az={rf[6]:+.4f}g")
    print(f"  Gyro        : wx={rf[1]:+.3f}  wy={rf[2]:+.3f}  wz={rf[3]:+.3f} dps")

    # Quaternion norm stability
    norms = [qnorm(r[8], r[9], r[10], r[11]) for r in imu_rows[first_real_idx:]]
    norm_min = min(norms); norm_max = max(norms)
    norm_mean = sum(norms) / len(norms)
    print(f"\n--- Quaternion Norm (samples {first_real_idx}..{n_imu-1}) ---")
    print(f"  min={norm_min:.6f}  max={norm_max:.6f}  mean={norm_mean:.6f}")
    if norm_max - norm_min > 0.01:
        print(f"  WARNING: norm range {norm_max-norm_min:.4f} > 0.01 -- possible instability")

    # Gyro statistics (full run)
    wx_vals = [r[1] for r in imu_rows]; wy_vals = [r[2] for r in imu_rows]; wz_vals = [r[3] for r in imu_rows]
    print(f"\n--- Gyro Range (all samples) ---")
    print(f"  wx: [{min(wx_vals):+.2f}, {max(wx_vals):+.2f}] dps")
    print(f"  wy: [{min(wy_vals):+.2f}, {max(wy_vals):+.2f}] dps")
    print(f"  wz: [{min(wz_vals):+.2f}, {max(wz_vals):+.2f}] dps")

    # Accel statistics
    ax_vals = [r[4] for r in imu_rows]; ay_vals = [r[5] for r in imu_rows]; az_vals = [r[6] for r in imu_rows]
    print(f"\n--- Accel Range (all samples) ---")
    print(f"  ax: [{min(ax_vals):+.4f}, {max(ax_vals):+.4f}] g")
    print(f"  ay: [{min(ay_vals):+.4f}, {max(ay_vals):+.4f}] g")
    print(f"  az: [{min(az_vals):+.4f}, {max(az_vals):+.4f}] g")

    # IMU temperature
    temps = [r[7] for r in imu_rows]
    print(f"\n--- IMU Temperature ---")
    print(f"  start={temps[0]:.2f}°C  end={temps[-1]:.2f}°C  "
          f"delta={temps[-1]-temps[0]:+.2f}°C  range=[{min(temps):.2f}, {max(temps):.2f}]")

    # Mag statistics
    if mag_rows:
        mx_vals = [r[1] for r in mag_rows]
        my_vals = [r[2] for r in mag_rows]
        mz_vals = [r[3] for r in mag_rows]
        mag_mags = [math.sqrt(r[1]**2 + r[2]**2 + r[3]**2) for r in mag_rows]
        print(f"\n--- Magnetometer ---")
        print(f"  Samples : {len(mag_rows)}")
        print(f"  X range : [{min(mx_vals):+.4f}, {max(mx_vals):+.4f}] G  "
              f"span={max(mx_vals)-min(mx_vals):.4f} G")
        print(f"  Y range : [{min(my_vals):+.4f}, {max(my_vals):+.4f}] G  "
              f"span={max(my_vals)-min(my_vals):.4f} G")
        print(f"  Z range : [{min(mz_vals):+.4f}, {max(mz_vals):+.4f}] G  "
              f"span={max(mz_vals)-min(mz_vals):.4f} G")
        print(f"  |B| : min={min(mag_mags):.4f}G  max={max(mag_mags):.4f}G  "
              f"mean={sum(mag_mags)/len(mag_mags):.4f}G")
        print(f"  NOTE: for sphere fitter (MagCal) to converge, need span >=")
        print(f"        ~2x hard-iron magnitude on all 3 axes. Rotate device in")
        print(f"        all orientations for 30+ seconds.")

    # Baro statistics
    if baro_rows:
        pressures = [r[1] for r in baro_rows]
        p_mean = sum(pressures) / len(pressures)
        p_first = baro_rows[0][1]
        # ISA: altitude = 44330 * (1 - (P/101325)^0.1903)
        alt_m = 44330.0 * (1.0 - (p_first / 101325.0) ** 0.1903)
        print(f"\n--- Barometer ---")
        print(f"  Samples  : {len(baro_rows)}")
        print(f"  Pressure : first={p_first:.1f} Pa  mean={p_mean:.1f} Pa  "
              f"range=[{min(pressures):.1f}, {max(pressures):.1f}] Pa")
        print(f"  ISA alt  : ~{alt_m:.0f} m MSL  (~{alt_m*3.281:.0f} ft)  (from first baro reading)")

    # IMU trace / sample output
    if args.imu_trace or args.imu_sample > 0:
        print(f"\n--- IMU Trace ---")
        print(f"{'idx':>5}  {'time':>8}  {'wx':>7}  {'wy':>7}  {'wz':>7}  "
              f"{'ax':>7}  {'ay':>7}  {'az':>7}  "
              f"{'qw':>7}  {'qx':>7}  {'qy':>7}  {'qz':>7}  {'|q|':>7}  "
              f"{'roll':>6}  {'pitch':>6}  {'yaw':>7}")
        step = 1 if args.imu_trace else args.imu_sample
        for i in range(0, n_imu, step):
            r = imu_rows[i]
            qw, qx, qy, qz = r[8], r[9], r[10], r[11]
            n = qnorm(qw, qx, qy, qz)
            if n > 0.5:
                ro, pi, ya = quat_to_euler_deg(qw, qx, qy, qz)
                euler_str = f"  {ro:+6.1f}  {pi:+6.1f}  {ya:+7.1f}"
            else:
                euler_str = "  (init)  (init)   (init)"
            print(f"{i:5d}  {r[0]:8.3f}  {r[1]:7.2f}  {r[2]:7.2f}  {r[3]:7.2f}  "
                  f"{r[4]:7.4f}  {r[5]:7.4f}  {r[6]:7.4f}  "
                  f"{qw:7.4f}  {qx:7.4f}  {qy:7.4f}  {qz:7.4f}  {n:7.4f}"
                  + euler_str)

    if args.mag_trace and mag_rows:
        print(f"\n--- MAG Trace ---")
        print(f"{'idx':>5}  {'time':>8}  {'mx':>8}  {'my':>8}  {'mz':>8}  {'|B|':>8}")
        for i, r in enumerate(mag_rows):
            mag = math.sqrt(r[1]**2 + r[2]**2 + r[3]**2)
            print(f"{i:5d}  {r[0]:8.3f}  {r[1]:8.4f}  {r[2]:8.4f}  {r[3]:8.4f}  {mag:8.4f}")

    print()


if __name__ == "__main__":
    main()
