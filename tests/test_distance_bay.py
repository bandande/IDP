import time
import math
import json
import sys
from machine import Pin, I2C

# ==================== HARDWARE CONFIGURATION ====================
# Both pins now sit on Hardware I2C Peripheral 1:
# Physical Pin 4  -> GP2  (I2C1 SDA)
# Physical Pin 15 -> GP11 (I2C1 SCL)
I2C_BUS_ID = 1
SDA_GPIO = 2   # Physical Pin 4
SCL_GPIO = 11  # Physical Pin 15
VL53L0X_ADDR = 0x29
# ================================================================

class VL53L0XHardwareDriver:
    """Hardware I2C Driver for DFRobot VL53L0X ToF Sensor."""
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self.stop_variable = 0
        self.init_hardware()

    def _read_u8(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def _write_u8(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))

    def init_hardware(self):
        """Hardware init sequence matching ST API / sensor_viewer specs."""
        model_id = self._read_u8(0xC0)
        if model_id != 0xEE:
            raise RuntimeError(f"Device at 0x29 returned Model ID {hex(model_id)}, expected 0xEE")

        # Standard ST API initialization sequence
        self._write_u8(0x88, 0x00)
        self._write_u8(0x80, 0x01)
        self._write_u8(0xFF, 0x01)
        self._write_u8(0x00, 0x00)
        self.stop_variable = self._read_u8(0x91)
        self._write_u8(0x00, 0x01)
        self._write_u8(0xFF, 0x00)
        self._write_u8(0x80, 0x00)

        # Static configuration
        self._write_u8(0x01, 0xE8)
        self._write_u8(0x0B, 0x01)  # Clear interrupt flag

    def read_distance_raw(self):
        """Triggers single-shot measurement and reads range status + distance."""
        try:
            # 1. Trigger single-shot measurement
            self._write_u8(0x80, 0x01)
            self._write_u8(0xFF, 0x01)
            self._write_u8(0x00, 0x00)
            self._write_u8(0x91, self.stop_variable)
            self._write_u8(0x00, 0x01)
            self._write_u8(0xFF, 0x00)
            self._write_u8(0x80, 0x00)
            self._write_u8(0x00, 0x01)  # SYSRANGE_START

            # 2. Wait for data ready bit on register 0x13
            ready = False
            for _ in range(50):
                if (self._read_u8(0x13) & 0x07) != 0:
                    ready = True
                    break
                time.sleep_ms(2)

            if not ready:
                return None, None

            # 3. Read 12-byte status block starting at RESULT_RANGE_STATUS (0x14)
            data = self.i2c.readfrom_mem(self.address, 0x14, 12)
            self._write_u8(0x0B, 0x01)  # Clear interrupt

            status_code = (data[0] & 0x78) >> 3
            dist_mm = (data[10] << 8) | data[11]

            return status_code, dist_mm
        except Exception:
            return None, None

    def read_distance_cm(self):
        """Returns distance in cm if range status is valid."""
        status_code, dist_mm = self.read_distance_raw()

        if status_code is None or dist_mm is None:
            return None

        # Filter out invalid values (8190/8191 = out of range / error)
        if dist_mm <= 10 or dist_mm >= 2000 or dist_mm in (8190, 8191):
            return None

        return dist_mm / 10.0


def verify_hardware_connection(i2c, address=0x29):
    """Verifies sensor presence on Hardware I2C1 bus."""
    print("=" * 68)
    print("       HARDWARE VERIFICATION: BAY TOF SENSOR CHECK            ")
    print("=" * 68)
    print(f"Scanning Hardware I2C(1) on Physical Pin 4 (GP2) & Pin 15 (GP11)...")

    devices = i2c.scan()

    if address not in devices:
        print("\n" + "!" * 68)
        print("  CRITICAL ERROR: SENSOR NOT DETECTED ON HARDWARE I2C1 BUS!")
        print("  -------------------------------------------------------------")
        print(f"  • Expected Address: 0x29 ({address})")
        print(f"  • Devices Found:    {[hex(d) for d in devices] if devices else 'NONE'}")
        print("  • Wiring Checklist:")
        print("    1. VCC -> 3.3V (Physical Pin 36)")
        print("    2. GND -> GND  (Physical Pin 3 or 8)")
        print("    3. SDA -> Physical Pin 4  (GP2)")
        print("    4. SCL -> Physical Pin 15 (GP11)")
        print("!" * 68 + "\n")
        raise SystemExit("PROGRAM TERMINATED: Hardware not detected.")

    model_id = i2c.readfrom_mem(address, 0xC0, 1)[0]
    print(f"  [✓] Hardware I2C1 Acknowledge Received at 0x29.")
    print(f"  [✓] Model ID Verified: {hex(model_id)}")
    print("  [✓] BAY TOF SENSOR IS ONLINE AND TRANSMITTING LIVE DATA!")
    print("=" * 68 + "\n")


def scan_for_duration(sensor, duration_sec=10.0):
    """Scans continuously for 10 seconds with live distance readout."""
    readings = []
    start_ticks = time.ticks_ms()
    target_ms = int(duration_sec * 1000)

    while time.ticks_diff(time.ticks_ms(), start_ticks) < target_ms:
        elapsed_sec = time.ticks_diff(time.ticks_ms(), start_ticks) / 1000.0
        remaining_sec = max(0.0, duration_sec - elapsed_sec)

        dist_cm = sensor.read_distance_cm()
        if dist_cm is not None:
            readings.append(dist_cm)
            live_str = f"{dist_cm:5.1f} cm"
        else:
            live_str = "  N/A   "

        pct = int((elapsed_sec / duration_sec) * 100)
        print(f"\r  [Live Distance: {live_str} | Time Left: {remaining_sec:4.1f}s ({pct:3d}%) | Samples: {len(readings):3d}]", end="")
        time.sleep_ms(30)

    print()

    if len(readings) < 5:
        return None

    # Interquartile Range (IQR) Outlier Filtering
    readings.sort()
    q1 = readings[len(readings) // 4]
    q3 = readings[(len(readings) * 3) // 4]
    iqr = q3 - q1
    filtered = [x for x in readings if (q1 - 1.5 * iqr) <= x <= (q3 + 1.5 * iqr)]

    if not filtered:
        filtered = readings

    mean_dist = sum(filtered) / len(filtered)
    variance = sum((x - mean_dist) ** 2 for x in filtered) / len(filtered)
    stdev = math.sqrt(variance)

    return {
        "mean": mean_dist,
        "stdev": stdev,
        "min": min(filtered),
        "max": max(filtered),
        "count": len(filtered)
    }


def run_bay_calibration_session():
    # 1. Initialize Hardware I2C peripheral 1
    i2c = I2C(I2C_BUS_ID, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=400000)

    # 2. Hardware verification check
    verify_hardware_connection(i2c, address=VL53L0X_ADDR)

    # 3. Driver setup
    print("Initializing VL53L0X hardware registers...", end="")
    sensor = VL53L0XHardwareDriver(i2c, address=VL53L0X_ADDR)
    print(" Done!\n")

    empty_bay_dataset = []
    box_present_dataset = []
    TARGET_TRIALS = 25

    print("      VL53L0X BAY DISTANCE CALIBRATION UTILITY (HARDWARE I2C)    ")
    print("=" * 68)
    print("Target Collection: 20 to 30 Trials (10-Second Continuous Laser Bursts)")
    print("-" * 68)

    trial_num = 1
    while True:
        empty_cnt = len(empty_bay_dataset)
        box_cnt = len(box_present_dataset)

        print(f"\n--- TRIAL [{trial_num}] (Empty Bay Scans: {empty_cnt} | Box Present Scans: {box_cnt}) ---")
        prompt = input("Is a BOX present in the bay for this trial? (y = Yes, n = No, q = Quit & Save): ").strip().lower()

        if prompt == 'q':
            break
        elif prompt not in ['y', 'yes', 'n', 'no']:
            print("Invalid entry! Type 'y' for Box Present, 'n' for Empty Bay, or 'q' to quit.")
            continue

        is_box_present = prompt in ['y', 'yes']
        condition_str = "BOX PRESENT" if is_box_present else "EMPTY BAY"

        print(f"Starting 10-second ToF collection for [{condition_str}]...")
        scan_result = scan_for_duration(sensor, duration_sec=10.0)

        if scan_result is None:
            print("  ⚠️ Scan Failed: Check hardware connections!")
            continue

        print(f"  ✓ Trial #{trial_num} Complete:")
        print(f"    • Mean Distance: {scan_result['mean']:.2f} cm")
        print(f"    • Std Deviation: ±{scan_result['stdev']:.2f} cm")
        print(f"    • Range Measured: {scan_result['min']:.1f} cm to {scan_result['max']:.1f} cm")

        if is_box_present:
            box_present_dataset.append(scan_result)
        else:
            empty_bay_dataset.append(scan_result)

        trial_num += 1

        if trial_num > TARGET_TRIALS:
            cont = input(f"\nCompleted {TARGET_TRIALS} trials! Finish calibration? (ENTER = Finish, 'c' = Continue): ").strip().lower()
            if cont != 'c':
                break

    if not empty_bay_dataset or not box_present_dataset:
        print("\n⚠️ Calibration incomplete: Must record at least ONE 'Empty Bay' and ONE 'Box Present' trial.")
        return

    # Statistical Aggregation
    empty_means = [d["mean"] for d in empty_bay_dataset]
    box_means = [d["mean"] for d in box_present_dataset]

    empty_avg = sum(empty_means) / len(empty_means)
    box_avg = sum(box_means) / len(box_means)

    empty_min, empty_max = min(empty_means), max(empty_means)
    box_min, box_max = min(box_means), max(box_means)

    empty_stdev = math.sqrt(sum((x - empty_avg) ** 2 for x in empty_means) / len(empty_means))
    box_stdev = math.sqrt(sum((x - box_avg) ** 2 for x in box_means) / len(box_means))

    decision_threshold = (empty_avg + box_avg) / 2.0
    hysteresis_band = abs(empty_avg - box_avg) * 0.15

    txt_report = f"""==================================================
      VL53L0X HOLDING BAY CALIBRATION REPORT      
==================================================
Sensor Type: Time-of-Flight Laser (DFRobot VL53L0X)
Hardware:    Physical Pin 4  (GP2  - Hardware I2C1 SDA)
             Physical Pin 15 (GP11 - Hardware I2C1 SCL)
Total Scans: {len(empty_bay_dataset) + len(box_present_dataset)} (10-second laser sampling bursts)

EMPTY BAY STATISTICS:
  Trial Count:    {len(empty_bay_dataset)}
  Mean Distance:  {empty_avg:.2f} cm
  Std Deviation:  ±{empty_stdev:.2f} cm
  Min Distance:   {empty_min:.2f} cm
  Max Distance:   {empty_max:.2f} cm

BOX PRESENT STATISTICS:
  Trial Count:    {len(box_present_dataset)}
  Mean Distance:  {box_avg:.2f} cm
  Std Deviation:  ±{box_stdev:.2f} cm
  Min Distance:   {box_min:.2f} cm
  Max Distance:   {box_max:.2f} cm

DECISION BOUNDARIES:
  Decision Threshold: {decision_threshold:.2f} cm
  Hysteresis Band:    ±{hysteresis_band:.2f} cm
  Box Surface Nearer: {box_avg < empty_avg}

RAW JSON PARAMS FOR ROBOT ALGORITHM:
{json.dumps({
    "sensor_type": "VL53L0X_Hardware_I2C",
    "bus_id": I2C_BUS_ID,
    "sda_gpio": SDA_GPIO,
    "scl_gpio": SCL_GPIO,
    "empty_bay_mean_cm": round(empty_avg, 2),
    "empty_bay_stdev_cm": round(empty_stdev, 2),
    "box_present_mean_cm": round(box_avg, 2),
    "box_present_stdev_cm": round(box_stdev, 2),
    "decision_threshold_cm": round(decision_threshold, 2),
    "hysteresis_band_cm": round(hysteresis_band, 2),
    "box_is_nearer": box_avg < empty_avg
})}
==================================================
"""

    filename = "bay_distance_calibration.txt"
    with open(filename, "w") as f:
        f.write(txt_report)

    print("\n\n" + "#" * 68)
    print("  COPY AND SAVE THE TEXT BLOCK BELOW TO A .TXT FILE ON YOUR COMPUTER")
    print("#" * 68 + "\n")
    print(txt_report)
    print("#" * 68)
    print(f"Saved to Pico internal flash storage as: '{filename}'")
    print("#" * 68 + "\n")


if __name__ == "__main__":
    run_bay_calibration_session()