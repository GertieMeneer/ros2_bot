import time
from rpi_ws281x import PixelStrip, Color

LED_COUNT = 8
LED_PIN = 12
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 255        #0-255
LED_INVERT = False 
LED_CHANNEL = 0

strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms / 1000.0)

if __name__ == '__main__':
    try:
        while True:
            print('Color wipe animations.')
            colorWipe(strip, Color(255, 0, 0))
            colorWipe(strip, Color(0, 255, 0))
            colorWipe(strip, Color(0, 0, 255))

    except KeyboardInterrupt:
        colorWipe(strip, Color(0, 0, 0), 10)