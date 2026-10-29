

def detect_magnet_pole(adc_pin, baseline=None, duration_sec=0.5):
    import time
    import math
    from machine import ADC, Pin
    """
    Reads the analog Hall sensor for ~0.5s and evaluates magnetic pole & confidence.
    
    :param adc_pin: GPIO pin number connected to sensor output (e.g. 26 for GP26).
    :param baseline: (Optional) Pre-calibrated ambient ADC baseline (LSB). 
                     If None, performs a fast 50ms ambient auto-calibration.
    :param duration_sec: (Optional) Sampling window duration in seconds (default: 0.5s).
    
    :return: Tuple of (pole: str, confidence: float)
             - pole: "N" or "S"
             - confidence: float between 0.0 and 1.0
    """
    adc = ADC(Pin(adc_pin))
    
    # Helper: Median pre-filter to drop spikes
    def read_median(count=5):
        buf = [adc.read_u16() for _ in range(count)]
        time.sleep_us(150)
        buf.sort()
        return float(buf[count // 2])

    # 1. Fast Baseline Auto-Calibration if not provided
    if baseline is None:
        init_samples = [read_median(5) for _ in range(10)]
        baseline = sum(init_samples) / len(init_samples)

    # Filter parameters & thresholds
    Q = 15.0       # Kalman process noise
    R = 100.0      # Measurement noise
    high_thresh = 55.0  # LSB threshold above ambient noise to trigger
    
    kalman_x = baseline
    kalman_p = R
    
    start_ticks = time.ticks_ms()
    target_ms = int(duration_sec * 1000)
    
    max_abs_delta = 0.0
    peak_delta = 0.0
    detected_state = "NEUTRAL"

    # 2. Continuous 0.5-Second Sampling Loop
    while time.ticks_diff(time.ticks_ms(), start_ticks) < target_ms:
        z = read_median(5)

        # 1D Kalman Filter step
        p_prior = kalman_p + Q
        k_gain = p_prior / (p_prior + R)
        kalman_x = kalman_x + k_gain * (z - kalman_x)
        kalman_p = (1.0 - k_gain) * p_prior

        delta = kalman_x - baseline
        abs_delta = abs(delta)

        if abs_delta > max_abs_delta:
            max_abs_delta = abs_delta
            peak_delta = delta

        # Pole Classification
        if abs_delta >= high_thresh:
            detected_state = "SOUTH" if delta > 0 else "NORTH"

        time.sleep_ms(10)

    # 3. Determine Pole String Output ("N" or "S")
    if detected_state == "NORTH":
        pole = "N"
    elif detected_state == "SOUTH":
        pole = "S"
    else:
        # Neutral state fallback: picks dominant polarity direction
        pole = "S" if peak_delta >= 0 else "N"

    # 4. Calculate Confidence (0.0 to 1.0)
    if detected_state == "NEUTRAL":
        confidence = 0.0
    else:
        # SNR-based scaling normalized to 1.0 max
        snr = max_abs_delta / high_thresh
        confidence = min(1.0, max(0.0, snr / 2.0))

    return pole, round(confidence, 2)


# ==================== EXAMPLE USAGE ====================
if __name__ == "__main__":
    # Call function on GP26 (Pin 31)
    pole, confidence = detect_magnet_pole(adc_pin=26)
    
    print(f"Detected Pole: {pole}")
    print(f"Confidence:    {confidence}")