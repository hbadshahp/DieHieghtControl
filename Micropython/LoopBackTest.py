from machine import Pin, UART
import time
import network
import ubinascii
from umqtt.simple import MQTTClient

# Modbus settings
MODBUS_DIR_PIN = Pin(23, Pin.OUT)
MODBUS_RX_PIN = 16
MODBUS_TX_PIN = 17
MODBUS_SERIAL_BAUD = 9600
uart = UART(2, baudrate=MODBUS_SERIAL_BAUD, tx=MODBUS_TX_PIN, rx=MODBUS_RX_PIN)

# Function prototypes
def modbus_pre_transmission():
    MODBUS_DIR_PIN.off()

def modbus_post_transmission():
    MODBUS_DIR_PIN.on()
    
modbus_pre_transmission()
uart.write(b'hello')
time.sleep(1)
response = uart.read(5)
print(f"Loopback response: {response}")
modbus_post_transmission()