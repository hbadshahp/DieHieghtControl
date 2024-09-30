# Libraries and Dependencies
from machine import Pin, UART
import network
import time
from umqttsimple import MQTTClient
import json
from umodbus.serial import Serial
from micropython import const

# Constants
MODBUS_SLAVE_ADDRESS = const(0x08)  # Modbus slave address
MODBUS_START_ADDRESS = const(0x00)  # Start reading from address 0x0000
MODBUS_REGISTER_QUANTITY = const(2)  # Read two registers (32-bit value)
RELAY_ACTIVATION_DURATION = 0.05  # seconds
DIE_SIGNAL_MIN = 250.0
DIE_SIGNAL_MAX = 350.0
HEIGHT_THRESHOLD = 0.125

# Define the pins for Modbus communication and relay pins
rtu_pins = (Pin(17), Pin(18))
RELAY_UP_PIN = Pin(21, Pin.OUT)
RELAY_DOWN_PIN = Pin(19, Pin.OUT)

# Initialize the Modbus object
m = Serial(
    baudrate=9600,
    data_bits=8,
    stop_bits=1,
    parity=None,
    pins=rtu_pins,
    ctrl_pin=Pin(15),
    uart_id=2
)

# Initialize UART timeout
# Assuming the timeout is in milliseconds
m._uart.init(timeout=2000)  # Set UART timeout to 2000 ms (2 seconds)

# WiFi settings
SSID = "raspi4-iiot"
PASSWORD = "iota2024"

# MQTT settings
MQTT_SERVER = "10.42.0.1"
MQTT_PORT = 1883
MQTT_USER = "npdtom"
MQTT_PASSWORD = "npd@tom"

MQTT_TOPIC_PUBLISH = "die_height"
MQTT_TOPIC_SUBSCRIBE = "die_signal"

# MQTT client setup
CLIENT_ID = "03"

# Global MQTT client variable
client = None

# Connect to WiFi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        timeout = 10  # seconds
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("Failed to connect to WiFi")
                return False
            print("Waiting for WiFi connection...")
            time.sleep(1)
    print("WiFi connected, IP address:", wlan.ifconfig()[0])
    return True

# MQTT callback function
def mqtt_callback(topic, msg):
    print(f"Message received: Topic: {topic}, Msg: {msg}")
    try:
        message = msg.decode('utf-8').strip()
        print(f"Decoded message: {message}")
        
        if message == "up":
            RELAY_UP_PIN.on()  # Activate the relay
            time.sleep(RELAY_ACTIVATION_DURATION)
            RELAY_UP_PIN.off()  # Deactivate the relay
            print("Up relay triggered")
        elif message == "down":
            RELAY_DOWN_PIN.on()  # Activate the relay
            time.sleep(RELAY_ACTIVATION_DURATION)
            RELAY_DOWN_PIN.off()  # Deactivate the relay
            print("Down relay triggered")
        else:
            die_signal = float(message)
            if die_signal < DIE_SIGNAL_MIN or die_signal > DIE_SIGNAL_MAX:
                print(f"Invalid input: {die_signal} is out of range ({DIE_SIGNAL_MIN}-{DIE_SIGNAL_MAX})")
                return
            
            success, current_height = readCurrentDieHeight()
            if not success:
                print("Failed to read Modbus registers")
                return
            
            difference = current_height - die_signal
            print(f"Die signal: {die_signal}, Current height: {current_height}, Difference: {difference}")
            
            if difference > HEIGHT_THRESHOLD:
                print("Difference exceeds threshold. Operating down relay.")
                operateRelay(RELAY_DOWN_PIN, die_signal, HEIGHT_THRESHOLD, is_upward=False)
            elif difference < -HEIGHT_THRESHOLD:
                print("Difference exceeds threshold. Operating up relay.")
                operateRelay(RELAY_UP_PIN, die_signal, HEIGHT_THRESHOLD, is_upward=True)
            else:
                print("Height is within acceptable range. No relay action needed.")
    except ValueError:
        print("Received message is not a valid float or expected command.")
    except Exception as e:
        print(f"Error in MQTT callback: {e}")

# Connect to MQTT broker
def connect_mqtt():
    global client
    try:
        client = MQTTClient(CLIENT_ID, MQTT_SERVER, port=MQTT_PORT, user=MQTT_USER, password=MQTT_PASSWORD)
        client.set_callback(mqtt_callback)
        client.connect()
        client.subscribe(MQTT_TOPIC_SUBSCRIBE)
        print(f"Connected to MQTT broker at {MQTT_SERVER} with client ID {CLIENT_ID}")
        return True
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        return False

# Function to read current die height
def readCurrentDieHeight():
    try:
        register_values = m.read_holding_registers(
            MODBUS_SLAVE_ADDRESS,
            MODBUS_START_ADDRESS,
            MODBUS_REGISTER_QUANTITY,
            signed=False
        )
        
        if register_values is None or len(register_values) != MODBUS_REGISTER_QUANTITY:
            print("Failed to read Modbus registers or incomplete data received")
            return False, 0.0
        
        # Combine the two 16-bit registers into a 32-bit unsigned integer
        register1 = register_values[0]
        register2 = register_values[1]

        combined_value = (register1 << 16) | register2

        # Convert to floating-point (assuming scaling factor of 100000)
        die_height = combined_value / 100000.0

        print(f"Current Die Height: {die_height:.2f}")
        return True, die_height

    except Exception as e:
        print(f"Error reading Modbus registers: {e}")
        return False, 0.0

# Operate relay
def operateRelay(relay_pin, target_height, threshold, is_upward):
    relay_pin.on()  # Activate the relay
    action = "up" if is_upward else "down"
    print(f"{action.capitalize()} relay activated to reach target height {target_height}")
    
    success, current_height = readCurrentDieHeight()
    
    while success:
        difference = target_height - current_height
        print(f"Operating Relay: Current Height = {current_height}, Target = {target_height}, Difference = {difference}")
        
        if (is_upward and difference <= threshold) or (not is_upward and difference >= -threshold):
            print("Threshold reached. Deactivating relay.")
            break
        
        time.sleep(RELAY_ACTIVATION_DURATION)  # Small delay to prevent rapid iterations
        success, current_height = readCurrentDieHeight()
    
    relay_pin.off()  # Deactivate the relay
    print(f"{action.capitalize()} relay deactivated. Target height reached.")

# Main loop
def main():
    # Connect to WiFi with retry mechanism
    while not connect_wifi():
        print("Retrying WiFi connection in 5 seconds...")
        time.sleep(5)
    
    # Connect to MQTT with retry mechanism
    while not connect_mqtt():
        print("Retrying MQTT connection in 5 seconds...")
        time.sleep(5)
    
    while True:
        try:
            client.check_msg()  # Check for new MQTT messages
            
            # Optional: Publish current die height periodically
            # Uncomment the following block if publishing is desired
            
            '''
            success, die_height = readCurrentDieHeight()
            if success:
                msg = json.dumps({"height": die_height})
                client.publish(MQTT_TOPIC_PUBLISH, msg)
                print(f"Published die height: {die_height}")
            else:
                print("Failed to read die height")
            time.sleep(1)
            '''
        
        except OSError as e:
            print(f"MQTT connection lost: {e}")
            while not connect_mqtt():
                print("Retrying MQTT connection in 5 seconds...")
                time.sleep(5)
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(5)

# Execute the main loop
if __name__ == "__main__":
    main()
