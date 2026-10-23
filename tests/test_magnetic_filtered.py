import time
import math
from machine import ADC, Pin

# ==================== HARDWARE CONFIGURATION ====================
# Magnetometer Output -> Pi Pico Pin 31 (GPIO26 / ADC0)
MAGNET_ADC_PIN = 26 
# ================================================================

class AnalogMagnetDetector:
    def __init__(self, adc_pin=26, oversample=64, alpha=0.08, sigma_mult=3.0):
        """
        :param adc_pin: GPIO pin connected to sensor output (GP26 / Pin 31)[cite: 3]
        :param oversample: Number of raw samples averaged per update to suppress ADC white noise
        :param alpha: Low-pass filter smoothing weight (0.05 = heavy filter, 0.20 = fast response)
        :param sigma_mult: Sensitivity threshold multiplier above background noise floor
        """
        self.adc = ADC(Pin(adc_pin))
        self.oversample = oversample
        self.alpha = alpha
        self.sigma_mult = sigma_mult
        
        self.baseline_mean = 0.0
        self.baseline_sigma = 0.0
        self.filtered_val = 0.0
        self.high_thresh = 0.0
        self.low_thresh = 0.0
        self.current_state = "NEUTRAL"

    def _read_raw_oversampled(self):
        """Averages N consecutive ADC readings to cut random noise by sqrt(N)."""
        total = 0
        for _ in range(self.oversample):
            total += self.adc.read_u16()
        return total / self.oversample

    def calibrate(self):
        """Interactive background noise calibration routine."""
        print("\n" + "=" * 55)
        print("          ANALOG MAGNET SENSOR AUTO-CALIBRATION")
        print("=" * 55)
        print("1. Ensure the magnet box is AT LEAST 30 cm away.")
        print("2. Ensure robot motors are stopped.")
        print("Starting baseline sampling in 3 seconds...", end="")
        for i in range(3, 0, -1):
            print(f" {i}...", end="")
            time.sleep(1)
        print("\nSampling ambient magnetic field...")

        samples = []
        for _ in range(150):
            samples.append(self._read_raw_oversampled())
            time.sleep_ms(10)

        # Compute Mean (Ambient offset voltage) & Standard Deviation (Noise floor)
        self.baseline_mean = sum(samples) / len(samples)
        variance = sum((x - self.baseline_mean) ** 2 for x in samples) / len(samples)
        self.baseline_sigma = math.sqrt(variance)
        
        # Set dynamic thresholds with hysteresis (minimum trigger floor of 50 LSB)
        self.high_thresh = max(self.sigma_mult * self.baseline_sigma, 50.0)
        self.low_thresh = 0.5 * self.high_thresh
        self.filtered_val = self.baseline_mean

        voltage = (self.baseline_mean / 65535.0) * 3.3
        print("-" * 55)
        print(f"CALIBRATION COMPLETE:")
        print(f"  Ambient ADC Baseline: {self.baseline_mean:.1f} LSB (~{voltage:.2f}V)")
        print(f"  Noise Floor (1σ):     {self.baseline_sigma:.2f} LSB")
        print(f"  Trigger Threshold:    ±{self.high_thresh:.1f} LSB above baseline")
        print("=" * 55 + "\n")

    def update(self):
        """Updates low-pass filter, hysteresis, and detects pole state."""
        raw = self._read_raw_oversampled()
        
        # Exponential Moving Average (EMA) low-pass filter
        self.filtered_val = (self.alpha * raw) + ((1.0 - self.alpha) * self.filtered_val)
        
        # Signal delta relative to calibrated ambient baseline
        delta = self.filtered_val - self.baseline_mean
        abs_delta = abs(delta)

        # State machine with hysteresis to eliminate output flickering
        if self.current_state == "NEUTRAL":
            if abs_delta >= self.high_thresh:
                self.current_state = "NORTH" if delta > 0 else "SOUTH"
        else:
            if abs_delta < self.low_thresh:
                self.current_state = "NEUTRAL"

        return self.current_state, delta


def render_terminal_bar(state, delta, max_scale=1200.0):
    """Renders a clean, single-line updating terminal gauge."""
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

    return f"\r{status} S <{bar_str}> N | Signal Delta: {delta:+7.1f} LSB"


# ==================== MAIN EXECUTION ====================
if __name__ == "__main__":
    # Initialize on GP26 (Pico Pin 31 / ADC0)[cite: 3]
    detector = AnalogMagnetDetector(adc_pin=26, oversample=64, alpha=0.08, sigma_mult=3.0)
    
    # Run calibration
    detector.calibrate()

    print("Live Pole Detection Active (Press Ctrl+C to stop)...\n")
    while True:
        state, delta = detector.update()
        print(render_terminal_bar(state, delta), end="")
        time.sleep_ms(40)