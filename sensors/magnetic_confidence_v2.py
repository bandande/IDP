import time
import math
from machine import ADC, Pin


def detect_magnetic_pole(adc_pin, baseline_data, run_time_sec=0.5):

    def _read_median_sample(adc, samples_count=5):
        """Stage 1: Median pre-filter to remove impulse EMI noise."""
        buf = [adc.read_u16() for _ in range(samples_count)]
        time.sleep_us(150)
        buf.sort()
        return float(buf[samples_count // 2])

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