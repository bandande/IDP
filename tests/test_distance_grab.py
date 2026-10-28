import time
import math
import json
import sys
from machine import Pin, SoftI2C

# ==================== HARDWARE CONFIGURATION ====================
# Physical Pin 4  -> GP2  (GPIO 2)  -> I2C SDA
# Physical Pin 22 -> GP17 (GPIO 17) -> I2C SCL
SDA_GPIO = 2   # Physical Pin 4
SCL_GPIO = 17  # Physical Pin 22
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

            # Filter out invalid out-of-range returns (20mm to 1200mm range)
            if dist_mm < 20 or dist_mm > 1200 or dist_mm == 8190:
                return None
            
            return dist_mm / 10.0  # Convert mm to cm
        except Exception:
            return None


def verify_hardware_connection(i2c, address=0x29):
    """
    Hardware Check: Scans the I2C bus and verifies communication 
    with the VL53L0X sensor. Raises an exception and halts if disconnected.
    """
    print("=" * 68)
    print("       HARDWARE VERIFICATION: GRAB TOF SENSOR CHECK           ")
    print("=" * 68)
    print(f"Scanning I2C Bus on Physical Pin 4 (SDA/GP{SDA_GPIO}) & Pin 22 (SCL/GP{SCL_GPIO})...")
    
    devices = i2c.scan()
    
    if address not in devices:
        print("\n" + "!" * 68)
        print("  CRITICAL ERROR: GRAB TOF SENSOR NOT DETECTED ON I2C BUS!")
        print("  -------------------------------------------------------------")
        print(f"  • Expected I2C Address: 0x29 ({address})")
        print(f"  • Addresses found on bus: {[hex(d) for d in devices] if devices else 'NONE'}")
        print("  • Troubleshooting Checklist:")
        print("    1. Verify VCC is connected to 3.3V and GND to Pico Ground.")
        print("    2. Check physical wiring: Pin 4 -> SDA, Pin 22 -> SCL.")
        print("    3. Ensure sensor XSHUT pin is pulled HIGH (or unconnected).")
        print("!" * 68 + "\n")
        
        # Halt execution completely
        raise SystemExit("PROGRAM TERMINATED: VL53L0X grab hardware connection failure.")

    # Read Model ID Register (0xC0) to verify active register access
    try:
        model_id = i2c.readfrom_mem(address, 0xC0, 1)[0]
        print(f"  [✓] Sensor acknowledge received at address 0x29.")
        print(f"  [✓] Hardware Register Read Test: Model ID = {hex(model_id)}")
        print("  [✓] GRAB TOF SENSOR IS ONLINE AND TRANSMITTING DATA!")
        print("=" * 68 + "\n")
        return True
    except Exception as e:
        print("\n" + "!" * 68)
        print(f"  CRITICAL ERROR: Sensor found at 0x29 but failed register read: {e}")
        print("!" * 68 + "\n")
        raise SystemExit("PROGRAM TERMINATED: Sensor communication fault.")


def scan_for_duration(sensor, duration_sec=10.0):
    """Scans continuously for ~10 seconds and applies IQR outlier filtering."""
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

    print()  # Newline after 10-second burst finishes

    if len(readings) < 10:
        return None  # Discard trial if too few valid returns

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


def run_grab_calibration_session():
    # 1. Initialize I2C Bus on Physical Pin 4 (GP2) and Pin 22 (GP17)
    i2c = SoftI2C(sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=400000)

    # 2. RUN MANDATORY HARDWARE CHECK BEFORE PROCEEDING
    verify_hardware_connection(i2c, address=VL53L0X_I2C_ADDR)

    # 3. Instantiate ToF Driver
    sensor = VL53L0XMinimalDriver(i2c, address=VL53L0X_I2C_ADDR)

    in_grab_range_dataset = []
    out_of_range_dataset = []
    TARGET_TRIALS = 25  # Target dataset size (20-30 trials)

    print("      VL53L0X BOX GRAB-DISTANCE CALIBRATION UTILITY        ")
    print("=" * 68)
    print(f"Hardware Pins: SDA = Physical Pin 4 (GP2) | SCL = Physical Pin 22 (GP17)")
    print(f"Target Collection: 20 to 30 Trials (10-Second Continuous Laser Bursts)")
    print("-" * 68)

    trial_num = 1
    while True:
        in_cnt = len(in_grab_range_dataset)
        out_cnt = len(out_of_range_dataset)

        print(f"\n--- TRIAL [{trial_num}] (In Grab Range: {in_cnt} | Out of Range: {out_cnt}) ---")
        input("Position the box relative to the gripper, then press ENTER to start 10s ToF scan...")

        print("Starting 10-second ToF data collection window...")
        print("-> Gently adjust box orientation/chassis tilt during scan to capture real movement noise...")

        scan_result = scan_for_duration(sensor, duration_sec=10.0)

        if scan_result is None:
            print("  ⚠️ Scan Failed: Sensor dropped communication! Check wiring on Physical Pins 4 & 22.")
            continue

        print(f"  ✓ 10-Second Scan Complete: Distance = {scan_result['mean']:.2f} cm (±{scan_result['stdev']:.2f} cm)")

        while True:
            choice = input("Is the box at the CORRECT DISTANCE TO GRAB? (y = Yes, n = No, r = Retry, q = Quit & Save): ").strip().lower()

            if choice == 'q':
                break
            elif choice in ['y', 'yes']:
                in_grab_range_dataset.append(scan_result)
                print(f"  [✓] Saved as IN GRAB RANGE (Total: {len(in_grab_range_dataset)})")
                trial_num += 1
                break
            elif choice in ['n', 'no']:
                out_of_range_dataset.append(scan_result)
                print(f"  [✗] Saved as OUT OF RANGE (Total: {len(out_of_range_dataset)})")
                trial_num += 1
                break
            elif choice in ['r', 'retry']:
                print("  -> Trial discarded. Retrying...")
                break
            else:
                print("Invalid input! Type 'y' for In Range, 'n' for Out of Range, 'r' to retry, or 'q' to quit.")

        if choice == 'q':
            break

        if trial_num > TARGET_TRIALS:
            cont = input(f"\nReached {TARGET_TRIALS} trials! Finish calibration? (ENTER = Finish, 'c' = Continue): ").strip().lower()
            if cont != 'c':
                break

    # Dataset validation
    if not in_grab_range_dataset or not out_of_range_dataset:
        print("\n⚠️ Calibration incomplete: Must record at least ONE 'In Grab Range' and ONE 'Out of Range' trial.")
        return

    # Statistical Aggregation
    grab_means = [d["mean"] for d in in_grab_range_dataset]
    out_means = [d["mean"] for d in out_of_range_dataset]

    grab_avg = sum(grab_means) / len(grab_means)
    out_avg = sum(out_means) / len(out_means)

    grab_min, grab_max = min(grab_means), max(grab_means)
    out_min, out_max = min(out_means), max(out_means)

    grab_stdev = math.sqrt(sum((x - grab_avg) ** 2 for x in grab_means) / len(grab_means))
    out_stdev = math.sqrt(sum((x - out_avg) ** 2 for x in out_means) / len(out_means))

    # Decision Boundaries
    decision_threshold = (grab_avg + out_avg) / 2.0
    hysteresis_band = abs(grab_avg - out_avg) * 0.15

    # Plain-Text Report Generation
    txt_report = f"""==================================================
        GRAB DISTANCE TOF CALIBRATION REPORT      
==================================================
Sensor Type: Time-of-Flight Laser (VL53L0X - Grab Sensor)
Hardware:    Physical Pin 4  (GP2  - I2C SDA)
             Physical Pin 22 (GP17 - I2C SCL)
Total Scans: {len(in_grab_range_dataset) + len(out_of_range_dataset)} (10-second laser sampling bursts)

IN GRAB RANGE METRICS (CORRECT DISTANCE):
  Trial Count:    {len(in_grab_range_dataset)}
  Mean Distance:  {grab_avg:.2f} cm
  Std Deviation:  ±{grab_stdev:.2f} cm
  Min Distance:   {grab_min:.2f} cm
  Max Distance:   {grab_max:.2f} cm

OUT OF RANGE METRICS (TOO FAR / NOT GRABBABLE):
  Trial Count:    {len(out_of_range_dataset)}
  Mean Distance:  {out_avg:.2f} cm
  Std Deviation:  ±{out_stdev:.2f} cm
  Min Distance:   {out_min:.2f} cm
  Max Distance:   {out_max:.2f} cm

DECISION BOUNDARIES:
  Max Grabbable Distance: {decision_threshold:.2f} cm
  Ideal Target Distance:  {grab_avg:.2f} cm
  Acceptable Grab Min:    {grab_min:.2f} cm
  Acceptable Grab Max:    {grab_max:.2f} cm
  Hysteresis Band:        ±{hysteresis_band:.2f} cm

RAW JSON PARAMS FOR ROBOT ALGORITHM:
{json.dumps({
    "sensor_type": "VL53L0X_GRAB",
    "sda_gpio": SDA_GPIO,
    "scl_gpio": SCL_GPIO,
    "ideal_grab_dist_cm": round(grab_avg, 2),
    "max_grab_dist_cm": round(decision_threshold, 2),
    "hysteresis_cm": round(hysteresis_band, 2),
    "grab_min_cm": round(grab_min, 2),
    "grab_max_cm": round(grab_max, 2),
    "grab_stdev_cm": round(grab_stdev, 2)
})}
==================================================
"""

    # 1. Save to Pico Flash Storage
    filename = "grab_distance_calibration.txt"
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
    run_grab_calibration_session()