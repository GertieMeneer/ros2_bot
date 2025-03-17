import RPi.GPIO as GPIO
import time

LDR_PIN = 18  # The GPIO pin connected to the LDR

def rc_time(pin):
    count = 0

    # Set pin as output and pull it LOW to discharge capacitor
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
    time.sleep(0.1)  # Short delay to discharge

    # Change pin to input mode and count charge time
    GPIO.setup(pin, GPIO.IN)
    while GPIO.input(pin) == GPIO.LOW:
        count += 1

    return count  # Higher count means dimmer light (longer charge time)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
try:
    while True:
        brightness = rc_time(LDR_PIN)
        print("Brightness Level:", brightness)
        time.sleep(1)  # Delay before next measurement

except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on Ctrl+C