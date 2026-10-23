import time
import math
from machine import ADC, Pin

class WeakMagnetDetector:
    def __init__(self, adc_pin=26, oversample=64, alpha=0.08, sigma_mult=3.5):
        """
        :param adc_pin: Pico ADC GPIO pin (26 = ADC0, 27 = ADC1, 28 = ADC2)
        :param oversample: Number of raw samples averaged per reading (reduces noise by sqrt(N))
        :param alpha: Exponential moving average filter weight (0.01 = heavy smoothing, 1.0 = no smoothing)
        :param sigma_mult: Sensitivity threshold (3.0-4.0x standard deviation prevents false triggers)
        """
        self.adc = ADC(Pin(adc_pin))
        self.oversample = oversample
        self.alpha = alpha
        self.sigma_mult = sigma_mult
        
        self.baseline_mean = 0.0
        self.baseline_sigma = 0.0
        self.filtered_value = 0.0
        
        # State tracking with hysteresis
        self.current_state = "NONE"
        self.high_threshold = 0.0
        self.low_threshold = 0.0

    def _read_raw_oversampled(self):
        """Burst-samples the ADC N times to decrease quantization & thermal noise."""
        total = 0
        for _ in range(self.oversample):
            total += self.adc.read_u16()
        return total / self.oversample

    def calibrate_baseline(self, num_samples=200):
        """
        Measures environmental background magnetic noise (Earth's field + circuit EMI).
        Ensure NO magnet is near the sensor during this calibration sequence!
        """
        print("Calibrating background noise floor... Keep magnet far away.")
        samples = []
        for _ in range(num_samples):
            samples.append(self._read_raw_oversampled())
            time.sleep_ms(10)

        # Calculate Mean (DC Offset)
        self.baseline_mean = sum(samples) / len(samples)
        
        # Calculate Standard Deviation (Noise Variance sigma)
        variance = sum((x - self.baseline_mean) ** 2 for x in samples) / len(samples)
        self.baseline_sigma = math.sqrt(variance)
        
        # Define dynamic detection limits
        self.high_threshold = self.sigma_mult * max(self.baseline_sigma, 1.0)
        self.low_threshold = 0.5 * self.high_threshold  # Hysteresis cutoff
        self.filtered_value = self.baseline_mean
        
        print(f"Calibration Complete:")
        print(f"  Baseline Mean:  {self.baseline_mean:.2f} LSB")
        print(f"  Noise Sigma:    {self.baseline_sigma:.2f} LSB")
        print(f"  Trigger Limit:  ±{self.high_threshold:.2f} LSB above baseline\n")

    def update(self):
        """
        Call this periodically in your main loop. Updates low-pass filter
        and returns detection state and signal strength.
        """
        raw_val = self._read_raw_oversampled()
        
        # Exponential Moving Average (EMA) Low-Pass Filtering
        self.filtered_value = (self.alpha * raw_val) + ((1.0 - self.alpha) * self.filtered_value)
        
        # Extract signal relative to baseline (Delta B)
        delta = self.filtered_value - self.baseline_mean
        abs_delta = abs(delta)
        
        # Pole & Orientation Detection with Hysteresis
        if self.current_state == "NONE":
            if abs_delta >= self.high_threshold:
                self.current_state = "NORTH_POLE" if delta > 0 else "SOUTH_POLE"
        else:
            # Drop back to NONE only when signal drops below hysteresis low limit
            if abs_delta < self.low_threshold:
                self.current_state = "NONE"

        return self.current_state, delta, abs_delta

# ==================== DEMONSTRATION MAIN LOOP ====================
if __name__ == "__main__":
    # Initialize on ADC0 (GPIO26)
    detector = WeakMagnetDetector(adc_pin=26, oversample=64, alpha=0.08, sigma_mult=3.5)
    
    # 1. Run baseline calibration on startup
    detector.calibrate_baseline(num_samples=150)
    
    print("Monitoring... Move magnet box near sensor.")
    while True:
        state, delta, strength = detector.update()
        
        if state != "NONE":
            print(f"[{state}] Strength: {strength:.1f} LSB (Raw Delta: {delta:+.1f})")
        else:
            print(f"[IDLE] Background Delta: {delta:+.1f} LSB", end="\r")
            
        time.sleep_ms(50)