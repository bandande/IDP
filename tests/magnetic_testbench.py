import time
import math
from machine import ADC, Pin

def _read_median_sample(adc, samples_count=5):
    """Stage 1: Median pre-filter to remove impulse EMI noise."""
    buf = [adc.read_u16() for _ in range(samples_count)]
    time.sleep_us(150)
    buf.sort()
    return float(buf[samples_count // 2])


def sample_magnetic_baseline(adc_pin, calibration_time_sec=1.0):
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


def detect_magnetic_pole(adc_pin, baseline_data, run_time_sec=0.5):
    """
    Runs the Kalman filtering pipeline over run_time_sec to detect magnet pole and confidence.
    
    :param adc_pin: GPIO pin number (e.g., 26 for GP26)
    :param baseline_data: Parameter dictionary returned by sample_magnetic_baseline()
    :param run_time_sec: Sampling window duration in seconds
    :return: Tuple of (pole_str: "N" or "S", confidence: float 0.0 to 1.0)
    """
    adc = ADC(Pin(adc_pin))
    
    # Extract baseline weights and thresholds
    baseline_mean = baseline_data["baseline_mean"]
    baseline_sigma = baseline_data["baseline_sigma"]
    R = baseline_data["R"]
    high_thresh = baseline_data["high_thresh"]
    low_thresh = baseline_data["low_thresh"]
    
    # Standard filtering constants
    Q = 15.0            # Process noise covariance
    alpha_drift = 0.0008 # Baseline drift rate during idle
    
    # State tracking
    kalman_x = baseline_mean
    kalman_p = R
    current_state = "NEUTRAL"
    hold_start_ticks = None
    
    max_confidence = 0.0
    detected_pole_state = "NEUTRAL"
    peak_delta = 0.0
    
    start_ticks = time.ticks_ms()
    target_ms = int(run_time_sec * 1000)
    
    while time.ticks_diff(time.ticks_ms(), start_ticks) < target_ms:
        # Stage 1: Median Pre-Filter
        z = _read_median_sample(adc, samples_count=5)
        
        # Stage 2: 1D Kalman Filter
        p_prior = kalman_p + Q
        k_gain = p_prior / (p_prior + R)
        kalman_x = kalman_x + k_gain * (z - kalman_x)
        kalman_p = (1.0 - k_gain) * p_prior
        
        delta = kalman_x - baseline_mean
        abs_delta = abs(delta)
        
        if abs_delta > abs(peak_delta):
            peak_delta = delta
            
        # Stage 3 & 4: State Machine & Adaptive Baseline Tracking
        prev_state = current_state
        if current_state == "NEUTRAL":
            if abs_delta >= high_thresh:
                current_state = "SOUTH" if delta > 0 else "NORTH"
            else:
                baseline_mean += alpha_drift * (kalman_x - baseline_mean)
        else:
            if abs_delta < low_thresh:
                current_state = "NEUTRAL"
                
        # Stage 5: Confidence Calculation
        if current_state != "NEUTRAL":
            if prev_state == "NEUTRAL":
                hold_start_ticks = time.ticks_ms()
                
            elapsed_sec = time.ticks_diff(time.ticks_ms(), hold_start_ticks) / 1000.0
            snr = abs_delta / max(baseline_sigma, 1.0)
            snr_boost = 1.0 + min(snr / 10.0, 1.0)
            
            # Confidence scaled from 0.0 to 1.0 over run_time_sec
            conf = min(1.0, (elapsed_sec / run_time_sec) * snr_boost)
            if conf > max_confidence:
                max_confidence = conf
                detected_pole_state = current_state
        else:
            hold_start_ticks = None
            
        time.sleep_ms(8)
        
    # Standardize return format
    if detected_pole_state == "NORTH":
        pole_str = "N"
    elif detected_pole_state == "SOUTH":
        pole_str = "S"
    else:
        pole_str = "S" if peak_delta >= 0 else "N"
        max_confidence = 0.0
        
    return pole_str, round(max_confidence, 2)

MAGNET_PIN = 26

# Run baseline sampling for 2.0 seconds at startup
print("Calibrating ambient magnetic field...")
magnetic_calibration_weights = sample_magnetic_baseline(MAGNET_PIN, calibration_time_sec=2.0)
print("Baseline Calibrated:", magnetic_calibration_weights)
x = input("Enter when ready to test")

# Run a 0.5-second scan window using stored weights
pole, confidence = detect_magnetic_pole(
    MAGNET_PIN, 
    magnetic_calibration_weights, 
    run_time_sec=0.5
)

if confidence >= 0.8:
    print(f"High-confidence detection: {pole} pole ({confidence * 100}%)")
    if pole == "N":
        # Trigger North Pole behavior
        pass
    elif pole == "S":
        # Trigger South Pole behavior
        pass
else:
    print("No strong magnetic field detected / Neutral state.")