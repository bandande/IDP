import time
import math
import json
import sys
from machine import Pin, SoftI2C

# ==================== HARDWARE CONFIGURATION ====================
# Physical Pin 1  -> GP0  (GPIO 0)  -> I2C SDA
# Physical Pin 15 -> GP11 (GPIO 11) -> I2C SCL
SDA_GPIO = 0   # Physical Pin 1
SCL_GPIO = 11  # Physical Pin 15
VL53L0X_I2C_ADDR = 0x29  # Standard I2C address for VL53L0X
# ================================================================

class VL53L0XMinimalDriver:
    """Minimal driver for VL53L0X Time-of-Flight (ToF) Distance Sensor."""
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self._init_sensor()

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val]))

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def _read_reg16(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return (data[0] << 8) | data[1]

    def _init_sensor(self):
        """Basic data-ready initialization sequence for VL53L0X."""
        # Standard power-on/reset sequence tuning
        self._write_reg(0x88, 0x00)
        self._write_reg(0x80, 0x01)
        self._write_reg(0xFF, 0x01)
        self._write_reg(0x00, 0x00)
        self._write_reg(0x91, 0x3C)
        self._write_reg(0x00, 0x01)
        self._write_reg(0xFF, 0x00)
        self._write_reg(0x80, 0x00)

    def read_distance_cm(self):
        """Triggers a single-shot ToF distance measurement in cm."""
        try:
            # Start single measurement
            self._write_reg(0x00, 0x01)
            
            # Wait for measurement data ready bit
            timeout = 100
            while (self._read_reg(0x13) & 0x07) == 0:
                time.sleep_ms(2)
                timeout -= 1
                if timeout <= 0:
                    return None  # Hardware timeout

            # Read 16-bit distance in millimeters at register 0x1E
            dist_mm = self._read_reg16(0x1E + 10)
            
            # Clear interrupt status
            self._write_reg(0x0B, 0x01)

            # Filter out invalid out-of-range sensor returns (20mm to 1200mm range)
            if dist_mm < 20 or dist_mm > 1200 or dist_mm == 8190:
                return None
            
            return dist_mm / 10.0  # Convert mm to cm
        except Exception:
            return None


def verify_hardware_connection(i2c, address=0x29):
    """
    Hardware Check: Scans the I2C bus and verifies data communication 
    with the VL53L0X sensor. Raises an exception and halts execution if disconnected.
    """
    print("=" * 68)
    print("       HARDWARE VERIFICATION: VL53L0X TOF SENSOR CHECK        ")
    print("=" * 68)
    print(f"Scanning I2C Bus on Physical Pin 1 (SDA/GP{SDA_GPIO}) & Pin 15 (SCL/GP{SCL_GPIO})...")
    
    devices = i2c.scan()
    
    if address not in devices:
        print("\n" + "!" * 68)
        print("  CRITICAL ERROR: VL53L0X SENSOR NOT DETECTED ON I2C BUS!")
        print("  -------------------------------------------------------------")
        print(f"  • Expected I2C Address: 0x29 ({address})")
        print(f"  • Addresses found on bus: {[hex(d) for d in devices] if devices else 'NONE'}")
        print("  • Troubleshooting Checklist:")
        print("    1. Verify VCC is connected to 3.3V and GND to Pico Ground.")
        print("    2. Check physical wiring: Pin 1 -> SDA, Pin 15 -> SCL.")
        print("    3. Ensure sensor XSHUT pin is pulled HIGH (or unconnected).")
        print("!" * 68 + "\n")
        
        # Halt execution completely
        raise SystemExit("PROGRAM TERMINATED: VL53L0X hardware connection failure.")

    # Read Model ID Register (0xC0) to ensure active register readout
    try:
        model_id = i2c.readfrom_mem(address, 0xC0, 1)[0]
        print(f"  [✓] Sensor acknowledge received at address 0x29.")
        print(f"  [✓] Hardware Register Read Test: Model ID = {hex(model_id)}")
        print("  [✓] SENSOR IS ONLINE AND TRANSMITTING DATA!")
        print("=" * 68 + "\n")
        return True
    except Exception as e:
        print("\n" + "!" * 68)
        print(f"  CRITICAL ERROR: Sensor found at 0x29 but failed register read: {e}")
        print("!" * 68 + "\n")
        raise SystemExit("PROGRAM TERMINATED: Sensor communication fault.")


def scan_for_duration(sensor, duration_sec=10.0):
    """Scans continuously for 10 seconds and applies IQR outlier filtering."""
    readings = []
    start_ticks = time.ticks_ms()
    target_ms = int(duration_sec * 1000)

    while time.ticks_diff(time.ticks_ms(), start_ticks) < target_ms:
        elapsed_sec = time.ticks_diff(time.ticks_ms(), start_ticks) / 1000.0
        remaining_sec = max(0.0, duration_sec - elapsed_sec)

        dist = sensor.read_distance_cm()
        if dist is not None:
            readings.append(dist)

        pct = int((elapsed_sec / duration_sec) * 100)
        print(f"\r  [ToF Laser Sampling... {remaining_sec:4.1f}s left ({pct:3d}%) | Valid Samples: {len(readings):3d}]", end="")
        time.sleep_ms(30)

    print()  # Newline when 10s completes

    if len(readings) < 10:
        return None  # Discard if too few returns

    # Interquartile Range (IQR) Outlier Filter
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
    # 1. Initialize I2C Bus on Physical Pin 1 (GP0) and Pin 15 (GP11)
    i2c = SoftI2C(sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=400000)

    # 2. RUN MANDATORY HARDWARE CHECK BEFORE PROCEEDING
    verify_hardware_connection(i2c, address=VL53L0X_I2C_ADDR)

    # 3. Instantiate Sensor Driver
    sensor = VL53L0XMinimalDriver(i2c, address=VL53L0X_I2C_ADDR)

    empty_bay_dataset = []
    box_present_dataset = []
    TARGET_TRIALS = 25  # Target total trials (20-30 range)

    print("      VL53L0X HOLDING BAY DISTANCE CALIBRATION UTILITY        ")
    print("=" * 68)
    print(f"Target Collection: 20 to 30 Trials (10-Second Continuous Laser Bursts)")
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
        print("-> Slightly adjust robot pitch/tilt during scan to simulate mechanical play...")

        scan_result = scan_for_duration(sensor, duration_sec=10.0)

        if scan_result is None:
            print("  ⚠️ Scan Failed: ToF sensor dropped frames! Re-check wiring.")
            continue

        print(f"  ✓ Trial #{trial_num} Complete:")
        print(f"    • Mean Distance: {scan_result['mean']:.2f} cm")
        print(f"    • Std Deviation: ±{scan_result['stdev']:.2f} cm")
        print(f"    • Samples Captured: {scan_result['count']}")

        if is_box_present:
            box_present_dataset.append(scan_result)
        else:
            empty_bay_dataset.append(scan_result)

        trial_num += 1

        if trial_num > TARGET_TRIALS:
            cont = input(f"\nCompleted {TARGET_TRIALS} trials! Finish calibration? (ENTER = Finish, 'c' = Continue): ").strip().lower()
            if cont != 'c':
                break

    # Dataset validation
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

    # Plain-Text Report Generation
    txt_report = f"""==================================================
      VL53L0X HOLDING BAY CALIBRATION REPORT      
==================================================
Sensor Type: Time-of-Flight Laser (VL53L0X)
Hardware:    Physical Pin 1  (GP0  - I2C SDA)
             Physical Pin 15 (GP11 - I2C SCL)
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
    "sensor_type": "VL53L0X",
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

    # 1. Save to Pico Flash Storage
    filename = "bay_distance_calibration.txt"
    with open(filename, "w") as f:
        f.write(txt_report)

    # 2. Output to Terminal for user host retrieval
    print("\n\n" + "#" * 68)
    print("  COPY AND SAVE THE TEXT BLOCK BELOW TO A .TXT FILE ON YOUR COMPUTER")
    print("#" * 68 + "\n")
    print(txt_report)
    print("#" * 68)
    print(f"Saved to Pico internal flash storage as: '{filename}'")
    print("Access in VS Code via: MicroPico -> File Explorer -> Download File")
    print("#" * 68 + "\n")


if __name__ == "__main__":
    run_bay_calibration_session()