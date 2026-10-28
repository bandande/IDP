import time
import math
import json
from machine import Pin, time_pulse_us

# ==================== HARDWARE CONFIGURATION ====================
# Physical Pin 4  -> GP2  (GPIO 2)  -> TRIG
# Physical Pin 22 -> GP17 (GPIO 17) -> ECHO
TRIG_GPIO = 2   # Physical Pin 4
ECHO_GPIO = 17  # Physical Pin 22
# ================================================================

class UltrasonicSensorDriver:
    """Hardware interface driver for HC-SR04 Ultrasonic sensor."""
    def __init__(self, trig_gpio=2, echo_gpio=17):
        self.trig = Pin(trig_gpio, Pin.OUT, value=0)
        self.echo = Pin(echo_gpio, Pin.IN)

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
        if dist is not None and 1.0 <= dist <= 300.0:
            readings.append(dist)
        time.sleep_ms(15)

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


def run_grab_calibration_session():
    sensor = UltrasonicSensorDriver(trig_gpio=TRIG_GPIO, echo_gpio=ECHO_GPIO)
    in_grab_range_dataset = []
    out_of_range_dataset = []
    
    TARGET_TRIALS = 30  # Target dataset size

    print("\n" + "=" * 65)
    print("      HIGH-PRECISION GRAB DISTANCE CALIBRATION UTILITY     ")
    print("=" * 65)
    print(f"Hardware Pins: TRIG = Physical Pin 4 (GP2) | ECHO = Physical Pin 22 (GP17)")
    print(f"Target Dataset: ~{TARGET_TRIALS} Trials (15 'In Range', 15 'Out of Range')")
    print("Instructions:")
    print(" 1. Adjust box position/tilt slightly between each trial.")
    print(" 2. Press ENTER to burst-scan 30 samples per trial.")
    print(" 3. Enter 'y' (Grabbable) or 'n' (Too Far/Out of Range).")
    print("-" * 65)

    trial_num = 1
    while True:
        in_cnt = len(in_grab_range_dataset)
        out_cnt = len(out_of_range_dataset)
        print(f"\n--- TRIAL [{trial_num}/{TARGET_TRIALS}] (In Range: {in_cnt} | Out of Range: {out_cnt}) ---")
        
        input("Position box & angle, then press ENTER to scan...")
        print("Scanning...", end="")

        scan_result = collect_scan_burst(sensor, samples_per_burst=30)

        if scan_result is None:
            print(f" [FAILED: No valid return! Check Physical Pins 4 & 22]")
            continue

        print(f" Done!\n  Measured Distance: {scan_result['mean']:.2f} cm (±{scan_result['stdev']:.2f} cm)")

        while True:
            choice = input("Is the box at the CORRECT DISTANCE TO GRAB? (y/n/r=retry): ").strip().lower()
            if choice in ['y', 'yes']:
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
                print("Invalid entry. Enter 'y', 'n', or 'r'.")

        print("-" * 45)
        if trial_num > TARGET_TRIALS:
            cont = input("Target reached! Finish calibration? (ENTER = Finish, 'c' = Keep scanning): ").strip().lower()
            if cont != 'c':
                break
        else:
            cont = input("Press ENTER for next trial ('q' = Finish early & save): ").strip().lower()
            if cont == 'q':
                break

    # Require at least one dataset from both states
    if not in_grab_range_dataset or not out_of_range_dataset:
        print("\n⚠️ Calibration incomplete: You must scan at least ONE 'IN GRAB RANGE' and ONE 'OUT OF RANGE' position.")
        return

    # Aggregate Statistics across all collected trials
    grab_means = [d["mean"] for d in in_grab_range_dataset]
    out_means = [d["mean"] for d in out_of_range_dataset]

    grab_avg = sum(grab_means) / len(grab_means)
    out_avg = sum(out_means) / len(out_means)

    grab_min, grab_max = min(grab_means), max(grab_means)
    out_min, out_max = min(out_means), max(out_means)

    # Calculate Standard Deviations across trial means
    grab_var = sum((x - grab_avg) ** 2 for x in grab_means) / len(grab_means)
    grab_stdev = math.sqrt(grab_var)

    out_var = sum((x - out_avg) ** 2 for x in out_means) / len(out_means)
    out_stdev = math.sqrt(out_var)

    # Decision boundaries
    decision_threshold = (grab_avg + out_avg) / 2.0
    hysteresis_band = abs(grab_avg - out_avg) * 0.15

    # Plain-Text Calibration Report
    txt_report = f"""==================================================
        GRAB DISTANCE CALIBRATION REPORT          
==================================================
Sensor Type: Ultrasonic (HC-SR04 - Grab Detector)
Trig Pin:    Physical Pin 4  (GP2)
Echo Pin:    Physical Pin 22 (GP17)
Total Scans: {len(in_grab_range_dataset) + len(out_of_range_dataset)} Trials

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
  Hysteresis Band:        ±{hysteresis_band:.2f} cm
  Ideal Target Distance:  {grab_avg:.2f} cm

RAW JSON PARAMS FOR REAL PROGRAM:
{json.dumps({
    "trig_gpio": TRIG_GPIO,
    "echo_gpio": ECHO_GPIO,
    "ideal_grab_dist_cm": round(grab_avg, 2),
    "max_grab_dist_cm": round(decision_threshold, 2),
    "hysteresis_cm": round(hysteresis_band, 2),
    "grab_min_cm": round(grab_min, 2),
    "grab_max_cm": round(grab_max, 2),
    "confidence_stdev_cm": round(grab_stdev, 2)
})}
==================================================
"""

    # 1. Save to Pico Flash Memory
    filename = "grab_distance_calibration.txt"
    with open(filename, "w") as f:
        f.write(txt_report)

    # 2. Output directly to Computer Terminal for easy copying
    print("\n\n" + "#" * 65)
    print("  COPY AND SAVE THE TEXT BLOCK BELOW TO A .TXT FILE ON YOUR COMPUTER")
    print("#" * 65 + "\n")
    print(txt_report)
    print("#" * 65)
    print(f"Report also saved internally to Pico Flash as: '{filename}'\n")


if __name__ == "__main__":
    run_grab_calibration_session()