import time
import math
from machine import ADC, Pin



def sample_magnetic_baseline(adc_pin, calibration_time_sec=1.0):

   
    def _read_median_sample(adc, samples_count=5):
        """Stage 1: Median pre-filter to remove impulse EMI noise."""
        buf = [adc.read_u16() for _ in range(samples_count)]
        time.sleep_us(150)
        buf.sort()
        return float(buf[samples_count // 2])
    
    
    """
    Samples ambient background magnetic field and calculates baseline parameters.
    
    :param adc_pin: GPIO pin number (e.g., 26 for GP26)
    :param calibration_time_sec: Time in seconds to run baseline sampling
    :return: Dictionary containing calibrated baseline parameters & noise thresholds
    """
    adc = ADC(Pin(adc_pin))
    samples = []
    
    start_ticks = time.ticks_ms()
    target_ms = int(calibration_time_sec * 1000)
    
    while time.ticks_diff(time.ticks_ms(), start_ticks) < target_ms:
        samples.append(_read_median_sample(adc, samples_count=5))
        time.sleep_ms(8)
        
    if not samples:
        samples = [_read_median_sample(adc, samples_count=5)]

    # Compute mean and noise variance (R)
    baseline_mean = sum(samples) / len(samples)
    variance = sum((x - baseline_mean) ** 2 for x in samples) / len(samples)
    baseline_sigma = math.sqrt(variance)
    
    R = max(variance, 10.0)
    high_thresh = max(3.0 * baseline_sigma, 55.0)  # Dynamic threshold above noise floor
    low_thresh = 0.70 * high_thresh               # Hysteresis threshold

    return {
        "baseline_mean": baseline_mean,
        "baseline_sigma": baseline_sigma,
        "R": R,
        "high_thresh": high_thresh,
        "low_thresh": low_thresh
    }
