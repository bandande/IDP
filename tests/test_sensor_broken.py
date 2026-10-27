import time
from machine import Pin

# ==================== HARDWARE CONFIGURATION ====================
# Middle Left Line Sensor -> Pi Pico Pin 27 (GPIO 21)
SENSOR_PIN_NUM = 21 
# ================================================================

def get_sensor_status(pin_num):
    """
    Checks if a physical sensor is connected, and returns its state.
    Returns: "DISCONNECTED", "CONNECTED_HIGH", or "CONNECTED_LOW"
    """
    # 1. Test with internal Pull-Down (~50kΩ to GND)
    p_down = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
    time.sleep_ms(5)
    val_down = p_down.value()

    # 2. Test with internal Pull-Up (~50kΩ to 3.3V)
    p_up = Pin(pin_num, Pin.IN, Pin.PULL_UP)
    time.sleep_ms(5)
    val_up = p_up.value()

    # If the reading follows the internal pulls, no external driver is present
    if val_down == 0 and val_up == 1:
        return "DISCONNECTED", None
    
    # If the reading stays high regardless of pull-down, sensor is driving HIGH
    elif val_down == 1 and val_up == 1:
        return "CONNECTED", 1

    # If the reading stays low regardless of pull-up, sensor is driving LOW
    elif val_down == 0 and val_up == 0:
        return "CONNECTED", 0

    return "UNKNOWN", None


# ==================== LIVE MONITORING LOOP ====================
print("=" * 60)
print("     LINE SENSOR CONNECTION & SIGNAL DIAGNOSTIC (GP21)    ")
print("=" * 60)
print("Press Ctrl+C to stop.\n")

try:
    while True:
        status, logic_val = get_sensor_status(SENSOR_PIN_NUM)

        if status == "DISCONNECTED":
            output_str = "\033[91m[ NO SENSOR DETECTED / WIRE DISCONNECTED ]\033[0m"
        else:
            if logic_val == 1:
                output_str = "\033[92m[ SENSOR CONNECTED | State: HIGH / DETECTED ]\033[0m"
            else:
                output_str = "\033[93m[ SENSOR CONNECTED | State: LOW  / CLEAR    ]\033[0m"

        print(f"\r{output_str}    ", end="")
        time.sleep_ms(100)

except KeyboardInterrupt:
    print("\n\nDiagnostic Stopped.")