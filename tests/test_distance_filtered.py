import time
import math
import json
from machine import Pin, time_pulse_us

# ==================== HARDWARE CONFIGURATION ====================
# Ultrasonic Distance Sensor (HC-SR04)
TRIG_PIN = 15  # GPIO 15 (Physical Pin 20)
ECHO_PIN = 1   # GPIO 1  (Physical Pin 2)
# ================================================================

class UltrasonicSensorDriver:
    """Hardware interface driver for HC-SR04 Ultrasonic sensor."""
    def __init__(self, trig_pin=15, echo_pin=1):
        self.trig = Pin(trig_pin, Pin.OUT, value=0)
        self.echo = Pin(echo_pin, Pin.IN)

    def read_distance_cm(self):
        """Returns single distance reading in cm."""
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        
        # Pulse timeout set to 30000us (~5m max range)
        duration = time_pulse_us(self.echo, 1, 30000)
        if duration < 0:
            return None  # Out of range / Timeout
            
        # Speed of sound: 343 m/s -> (duration / 2) * 0.0343
        return (duration / 2.0) * 0.0343


def collect_scan_burst(sensor, samples_per_burst=30):
    """Collects N readings, filters outliers using median/IQR, and returns statistics."""
    readings = []
    for _ in range(samples_per_burst):
        dist = sensor.read_distance_cm()
        if dist is not None and 2.0 <= dist <= 300.0:
            readings.append(dist)
        time.sleep_ms(20)

    if len(readings) < 5:
        return None  # Insufficient valid readings

    readings.sort()
    # Interquartile range (IQR) outlier rejection
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


def run_calibration_session():
    sensor = UltrasonicSensorDriver(trig_pin=TRIG_PIN, echo_pin=ECHO_PIN)
    empty_bay_dataset = []
    box_present_dataset = []

    print("\n" + "=" * 65)
    print("      ULTRASONIC DISTANCE SENSOR CALIBRATION UTILITY       ")
    print("=" * 65)
    print(f"Hardware Pins: TRIG = GP{TRIG_PIN} | ECHO = GP{ECHO_PIN}")
    print("Instructions:")
    print("1. Keep the robot in its fixed testing stance facing the bay.")
    print("2. Tilt the sensor slightly between scans to simulate real running.")
    print("3. Collect multiple scans for both EMPTY bay and BOX PRESENT states.")
    print("-" * 65)

    trial_num = 1
    while True:
        print(f"\n--- TRIAL #{trial_num} ---")
        input("Position bay condition & sensor tilt, then press ENTER to scan...")
        print("Scanning target bay...", end="")

        scan_result = collect_scan_burst(sensor, samples_per_burst=30)

        if scan_result is None:
            print(" [FAILED: No valid return! Check wiring on GP15/GP1]")
            continue

        print(f" Done!\n  Measured Distance: {scan_result['mean']:.2f} cm (±{scan_result['stdev']:.2f} cm)")

        while True:
            choice = input("Is a BOX present in the bay? (y = Yes, n = No, r = Retry scan): ").strip().lower()
            if choice in ['y', 'yes']:
                box_present_dataset.append(scan_result)
                print(f"  -> Saved as BOX PRESENT trial #{len(box_present_dataset)}")
                trial_num += 1
                break
            elif choice in ['n', 'no']:
                empty_bay_dataset.append(scan_result)
                print(f"  -> Saved as EMPTY BAY trial #{len(empty_bay_dataset)}")
                trial_num += 1
                break
            elif choice in ['r', 'retry']:
                print("  -> Trial discarded. Retrying...")
                break
            else:
                print("Invalid entry. Type 'y', 'n', or 'r'.")

        print("-" * 45)
        cont = input("Scan another position/tilt? (ENTER = Continue, 'q' = Finish & Save): ").strip().lower()
        if cont == 'q':
            break

    # Require at least one dataset from both classes
    if not empty_bay_dataset or not box_present_dataset:
        print("\n⚠️ Calibration incomplete: You must scan at least ONE 'EMPTY' and ONE 'BOX PRESENT' state.")
        return

    # Aggregate Statistics
    empty_means = [d["mean"] for d in empty_bay_dataset]
    box_means = [d["mean"] for d in box_present_dataset]

    empty_avg = sum(empty_means) / len(empty_means)
    box_avg = sum(box_means) / len(box_means)

    empty_min, empty_max = min(empty_means), max(empty_means)
    box_min, box_max = min(box_means), max(box_means)

    # Decision threshold calculation (midpoint between distributions)
    decision_threshold = (empty_avg + box_avg) / 2.0
    hysteresis_band = abs(empty_avg - box_avg) * 0.15

    # Generate Plain-Text Calibration Report
    txt_report = f"""==================================================
           DISTANCE CALIBRATION REPORT            
==================================================
Sensor Type: Ultrasonic (HC-SR04)
Trig Pin:    GP{TRIG_PIN}
Echo Pin:    GP{ECHO_PIN}
Total Scans: {len(empty_bay_dataset) + len(box_present_dataset)}

EMPTY BAY METRICS:
  Mean Distance: {empty_avg:.2f} cm
  Min Distance:  {empty_min:.2f} cm
  Max Distance:  {empty_max:.2f} cm
  Samples:       {len(empty_bay_dataset)}

BOX PRESENT METRICS:
  Mean Distance: {box_avg:.2f} cm
  Min Distance:  {box_min:.2f} cm
  Max Distance:  {box_max:.2f} cm
  Samples:       {len(box_present_dataset)}

DECISION THRESHOLDS:
  Decision Threshold: {decision_threshold:.2f} cm
  Hysteresis Band:    ±{hysteresis_band:.2f} cm
  Box Is Closer:      {box_avg < empty_avg}

RAW JSON PARAMS FOR REAL PROGRAM:
{json.dumps({
    "trig_pin": TRIG_PIN,
    "echo_pin": ECHO_PIN,
    "empty_mean_cm": round(empty_avg, 2),
    "box_mean_cm": round(box_avg, 2),
    "threshold_cm": round(decision_threshold, 2),
    "hysteresis_cm": round(hysteresis_band, 2),
    "box_is_closer": box_avg < empty_avg
})}
==================================================
"""

    # 1. Save to Pico Flash Memory
    filename = "distance_calibration.txt"
    with open(filename, "w") as f:
        f.write(txt_report)

    # 2. Output directly to Computer Terminal
    print("\n\n" + "#" * 65)
    print("  COPY AND SAVE THE TEXT BLOCK BELOW TO A .TXT FILE ON YOUR COMPUTER")
    print("#" * 65 + "\n")
    print(txt_report)
    print("#" * 65)
    print(f"Report also saved internally to Pico Flash as: '{filename}'\n")


if __name__ == "__main__":
    run_calibration_session()