"""
read_magcal.py - Inspect a MAGCAL.BIN file written by the FlySight 2 firmware.

Layout written by mag_cal.c (MagCal_FileData_t, ARM Cortex-M4, little-endian):
  Offset  Size  Type     Field
  0       12    float[3] hi_gauss  (hard iron X, Y, Z in gauss)
  12       1    uint8_t  quality   (0=UNKNOWN, 1=POOR, 2=OK, 3=GOOD)
  13       3    padding  (struct alignment to 4 bytes)
  Total = 16 bytes

Usage:
  python read_magcal.py MAGCAL.BIN
  python read_magcal.py              (reads MAGCAL.BIN in current directory)
"""

import struct
import sys
import os

QUALITY_NAMES = {0: "UNKNOWN", 1: "POOR", 2: "OK", 3: "GOOD"}
EXPECTED_SIZE = 16  # sizeof(MagCal_FileData_t) on ARM with 4-byte alignment


def read_magcal(path):
    size = os.path.getsize(path)
    if size < 13:
        print(f"ERROR: file too small ({size} bytes, need at least 13)")
        sys.exit(1)
    if size not in (13, 16):
        print(f"WARNING: unexpected file size {size} bytes (expected 13 or 16)")

    with open(path, "rb") as f:
        raw = f.read()

    # Unpack 3 floats + 1 uint8 from first 13 bytes (ignore trailing padding)
    hi_x, hi_y, hi_z, quality_byte = struct.unpack_from("<3fB", raw, 0)

    quality_name = QUALITY_NAMES.get(quality_byte, f"INVALID({quality_byte})")
    valid = quality_byte in (2, 3)  # OK or GOOD

    print(f"File   : {path}  ({size} bytes)")
    print(f"Quality: {quality_byte} = {quality_name}{'  ✓ applied to fusion' if valid else '  ✗ not applied to fusion (need OK or GOOD)'}")
    print(f"Hard iron (gauss):")
    print(f"  X = {hi_x:+.6f} G  ({hi_x * 100:.3f} uT)")
    print(f"  Y = {hi_y:+.6f} G  ({hi_y * 100:.3f} uT)")
    print(f"  Z = {hi_z:+.6f} G  ({hi_z * 100:.3f} uT)")
    print(f"  |HI| = {(hi_x**2 + hi_y**2 + hi_z**2)**0.5:.6f} G")

    print()
    print("Raw hex:", raw[:13].hex(" "))


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "MAGCAL.BIN"
    if not os.path.exists(path):
        print(f"ERROR: file not found: {path}")
        sys.exit(1)
    read_magcal(path)
