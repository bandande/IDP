import time
import math
from machine import ADC, Pin

# ==================== HARDWARE CONFIGURATION ====================
# Magnetometer Output -> Pi Pico Pin 31 (GPIO26 / ADC0)[cite: 3]
MAGNET_ADC_PIN = 26 
# ================================================================

class AdvancedAnalogMagnetDetector:
    def __init__(self, adc_pin=26, process_noise_q=15.0, sigma_mult=3.0, 
                 alpha_drift=0.0008, hold_target_sec=2.0):
        """
        :param adc_pin: GPIO pin connected to sensor output (GP26 / Pin 31)[cite: 3]
        :param process_noise_q: Kalman process variance (higher = faster motion response)
        :param sigma_mult: Dynamic detection limit multiplier above ambient noise floor
        :param alpha_drift: Continuous online baseline tracking rate during NEUTRAL state
        :param hold_target_sec: Required continuous hold duration (in seconds) to reach 100% confidence
        """
        self.adc = ADC(Pin(adc_pin))
        self.Q = process_noise_q
        self.sigma_mult = sigma_mult
        self.alpha_drift = alpha_drift
        self.hold_target_sec = hold_target_sec
        
        # Calibration & State variables
        self.baseline_mean = 0.0
        self.baseline_sigma = 0.0
        self.high_thresh = 0.0
        self.low_thresh = 0.0
        self.current_state = "NEUTRAL"

        # Kalman Filter initial states
        self.kalman_x = 0.0  # State estimate
        self.kalman_p = 1.0  # Error covariance
        self.R = 100.0       # Measurement noise variance

        # Hold & Confidence Tracking variables
        self.hold_start_ticks = None
        self.confidence = 0.0
        self.confirmed_latched = False

    def _read_median_sample(self, samples_count=5):
        """Stage 1: Median filter to eliminate impulse EMI spikes."""
        buf = []
        for _ in range(samples_count):
            buf.append(self.adc.read_u16())
            time.sleep_us(150)
        buf.sort()
        return float(buf[samples_count // 2])

    def calibrate(self):
        """Interactive background noise auto-calibration routine."""
        print("\n" + "=" * 55)
        print("      HYBRID KALMAN + MEDIAN MAGNET SENSOR CALIBRATION")
        print("=" * 55)
        print("1. Keep target magnet AT LEAST 30 cm away.")
        print("2. Ensure robot motors are stopped.")
        print("Starting baseline calibration in 3 seconds...", end="")
        for i in range(3, 0, -1):
            print(f" {i}...", end="")
            time.sleep(1)
        print("\nSampling ambient magnetic field noise...")

        samples = []
        for _ in range(120):
            samples.append(self._read_median_sample())
            time.sleep_ms(8)

        # Compute initial mean & noise variance (R)
        self.baseline_mean = sum(samples) / len(samples)
        variance = sum((x - self.baseline_mean) ** 2 for x in samples) / len(samples)
        self.baseline_sigma = math.sqrt(variance)
        
        self.R = max(variance, 10.0)
        self.kalman_x = self.baseline_mean
        self.kalman_p = self.R

        # Dynamic thresholds with hysteresis
        self.high_thresh = max(self.sigma_mult * self.baseline_sigma, 55.0)
        self.low_thresh = 0.70 * self.high_thresh

        voltage = (self.baseline_mean / 65535.0) * 3.3
        print("-" * 55)
        print(f"CALIBRATION COMPLETE:")
        print(f"  Ambient Baseline:   {self.baseline_mean:.1f} LSB (~{voltage:.2f}V)")
        print(f"  Noise Floor (1σ):   {self.baseline_sigma:.2f} LSB (R = {self.R:.1f})")
        print(f"  Trigger Threshold:  ±{self.high_thresh:.1f} LSB above baseline")
        print("=" * 55 + "\n")

    def update(self):
        """Processes filtering pipeline, state detection, and confidence calculation."""
        # 1. Stage 1: Median Pre-Filter
        z = self._read_median_sample(samples_count=5)

        # 2. Stage 2: 1D Kalman Filter
        p_prior = self.kalman_p + self.Q
        k_gain = p_prior / (p_prior + self.R)
        self.kalman_x = self.kalman_x + k_gain * (z - self.kalman_x)
        self.kalman_p = (1.0 - k_gain) * p_prior

        # Delta calculation relative to ambient baseline
        delta = self.kalman_x - self.baseline_mean
        abs_delta = abs(delta)

        # 3. Stage 3: Pole Detection State Machine (Flipped Polarity Logic)
        prev_state = self.current_state
        if self.current_state == "NEUTRAL":
            if abs_delta >= self.high_thresh:
                self.current_state = "SOUTH" if delta > 0 else "NORTH"
            else:
                # Stage 4: Adaptive Baseline Drift Tracking (only while NEUTRAL)
                self.baseline_mean += self.alpha_drift * (self.kalman_x - self.baseline_mean)
        else:
            if abs_delta < self.low_thresh:
                self.current_state = "NEUTRAL"

        # 4. Stage 5: Box Hold & Confidence Tracking Engine
        confirmation_event = None

        if self.current_state != "NEUTRAL":
            # State just entered: start timer
            if prev_state == "NEUTRAL":
                self.hold_start_ticks = time.ticks_ms()
                self.confidence = 0.0
                self.confirmed_latched = False

            # Calculate continuous hold duration & SNR
            elapsed_sec = time.ticks_diff(time.ticks_ms(), self.hold_start_ticks) / 1000.0
            snr = abs_delta / max(self.baseline_sigma, 1.0)
            
            # Confidence formula scaling with time and signal strength
            snr_boost = 1.0 + min(snr / 10.0, 1.0)
            self.confidence = min(100.0, (elapsed_sec / self.hold_target_sec) * snr_boost * 100.0)

            # Check if 100% confidence threshold reached for the first time in this hold
            if self.confidence >= 100.0 and not self.confirmed_latched:
                self.confirmed_latched = True
                confirmation_event = {
                    "pole": self.current_state,
                    "confidence": 100.0,
                    "hold_duration": elapsed_sec,
                    "snr": snr,
                    "delta": delta
                }
        else:
            # Reset hold metrics when box is dropped / removed
            self.hold_start_ticks = None
            self.confidence = 0.0
            self.confirmed_latched = False

        return self.current_state, delta, confirmation_event


def render_terminal_bar(state, delta, max_scale=1200.0):
    """Renders the standard single-line updating terminal gauge (Unchanged)."""
    norm = max(-1.0, min(1.0, delta / max_scale))
    center = 15
    bar_pos = int(center + (norm * center))

    gauge = list("-------------------------------")
    gauge[center] = "|"  # Zero center mark
    gauge[bar_pos] = "█"
    bar_str = "".join(gauge)

    if state == "NORTH":
        status = "[ NORTH POLE DETECTED ]"
    elif state == "SOUTH":
        status = "[ SOUTH POLE DETECTED ]"
    else:
        status = "[    NEUTRAL / IDLE   ]"

    return f"\r{status} N <{bar_str}> S | Signal Delta: {delta:+7.1f} LSB"


def emit_confirmation_banner(event):
    """Prints a new dedicated output banner when a held box pole is confirmed."""
    pole = event["pole"]
    duration = event["hold_duration"]
    snr = event["snr"]
    delta = event["delta"]

    # Newline ensures it prints cleanly above the active updating bar
    print("\n\n" + "★" * 60)
    print(f"  >>> HELD BOX POLE CONFIRMED: [ {pole} POLE ] <<<")
    print(f"  • Confidence:     100.0% (VERIFIED)")
    print(f"  • Duration Held:  {duration:.2f} seconds")
    print(f"  • Signal SNR:     {snr:.1f}x noise floor (Delta: {delta:+.1f} LSB)")
    print("★" * 60 + "\n")


# ==================== MAIN EXECUTION ====================
if __name__ == "__main__":
    # Initialize with a 2.0-second hold requirement for 100% confirmation
    detector = AdvancedAnalogMagnetDetector(
        adc_pin=MAGNET_ADC_PIN, 
        process_noise_q=15.0, 
        sigma_mult=3.0, 
        alpha_drift=0.0008,
        hold_target_sec=2.0
    )
    
    detector.calibrate()

    print("Live Pole Detection Active (Press Ctrl+C to stop)...\n")
    while True:
        state, delta, confirmation_event = detector.update()
        
        # New Output: Emit event output if box hold confidence threshold is reached
        if confirmation_event is not None:
            emit_confirmation_banner(confirmation_event)

        # Existing Output: Keep real-time updating terminal gauge unchanged
        print(render_terminal_bar(state, delta), end="")
        time.sleep_ms(15)