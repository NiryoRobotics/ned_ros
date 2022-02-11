import RPi.GPIO as GPIO
from threading import Timer


def enable_led_ring(bcm_pin):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(bcm_pin, GPIO.OUT)
    GPIO.output(bcm_pin, GPIO.HIGH)

    t = Timer(1, GPIO.output, args=[bcm_pin, GPIO.LOW])
    t.start()
