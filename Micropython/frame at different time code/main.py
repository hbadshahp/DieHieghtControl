# Libraries and Dependencies
from machine import Pin
import network
import time
from umqttsimple import MQTTClient
import json
from umodbus.serial import Serial
from machine import UART




# Define the pins for Modbus communication and relay pins
rtu_pins = (Pin(17), Pin(18))
RELAY_UP_PIN = Pin(21, Pin.OUT)
RELAY_DOWN_PIN = Pin(19, Pin.OUT)

# Initialize the Modbus object
m = Serial(baudrate=9600, data_bits=8, stop_bits=1, parity=None, pins=rtu_pins, ctrl_pin=Pin(15), uart_id=2)


# Increase timeout for Modbus response
m._uart.init(timeout=300)  # Set UART timeout to 2000 ms (2 seconds)
#m._uart.init()


# WiFi settings
ssid = "raspi4-iiot"
password = "iota2024"

# MQTT settings
mqtt_server = "10.42.0.1"
mqtt_port = 1883
mqtt_user = "npdtom"
mqtt_password = "npd@tom"

mqtt_topic = "die_height"
mqtt_subscribe_topic = "die_signal"

# MQTT client setup
client_id = "03"



# Connect to WiFi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(ssid, password)
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
    message = msg.decode('utf-8')
    print(message)
    
    if message == "up":
        
        RELAY_UP_PIN.on()  # Activate the relay
        time.sleep(0.05) 
        RELAY_UP_PIN.off()  # Deactivate the relay
    elif message == "down":
        
        RELAY_DOWN_PIN.on()  # Activate the relay
        time.sleep(0.05) 
        RELAY_DOWN_PIN.off()  # Deactivate the relay
    else:
        die_signal = float(message)
        if die_signal < 250.0 or die_signal > 350.0:
            print("Invalid input: Out of range")
            return
        
        success, current_height = readCurrentDieHeight()
        if not success:
            print("Failed to read Modbus registers")
            return
        
        difference = current_height - die_signal
        threshold = 0.125
        
        if difference > threshold:
            print("Operating down relay")
            operateRelay(RELAY_DOWN_PIN, die_signal, threshold, False)
        elif difference < -threshold:
            print("Operating up relay")
            operateRelay(RELAY_UP_PIN, die_signal, threshold, True)
        else:
            print("Height is within acceptable range")

# Connect to MQTT broker
def connect_mqtt():
    global client
    try:
        client = MQTTClient(client_id, mqtt_server, port=mqtt_port, user=mqtt_user, password=mqtt_password)
        client.set_callback(mqtt_callback)
        client.connect()
        client.subscribe(mqtt_subscribe_topic)
        print(f"Connected to MQTT broker at {mqtt_server} with client ID {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        return False
    return True
    
# Function to read current die height
def readCurrentDieHeight():
    slave_addr = const(0x08)  # Modbus slave address (updated to 8)
    starting_address = const(0x00)  # Start reading from address 0x0000
    register_quantity = const(2)  # Read two registers (32-bit value)

    # Try reading the holding registers
    try:
        #time.sleep(0.2)  # Small delay to give slave time to respond
        register_values = m.read_holding_registers(slave_addr, starting_address, register_quantity, signed=False)
        
        if register_values is None or len(register_values) != register_quantity:
            print("Failed to read Modbus registers")
            return False, 0.0
        
        # Combine the two 16-bit registers into a 32-bit unsigned integer
        register1 = register_values[0]
        register2 = register_values[1]

        combined_value = (register1 << 16) | register2

        # Convert to floating-point
        die_height = combined_value / 100000.0

        print(f"Current Die Height: {die_height:.2f}")
        return True, die_height

    except Exception as e:
        print(f"Error reading Modbus registers: {e}")
        return False, 0.0

# Operate relay
def operateRelay(relay_pin, target_height, threshold, is_upward):
    relay_pin.on()  # Activate the relay
    print(f"Relay {'up' if is_upward else 'down'} activated to reach target height {target_height}")

    success, current_height = readCurrentDieHeight()
    
    while success:
        difference = target_height - current_height
        print(f"Operating Relay: Current Height = {current_height}, Target = {target_height}, Difference = {difference}")
        
        if (is_upward and difference <= threshold) or (not is_upward and difference >= -threshold):
            print("Threshold reached. Deactivating relay.")
            break
        
        time.sleep(0.05)  # Small delay to prevent rapid iterations
        success, current_height = readCurrentDieHeight()
    
    relay_pin.off()  # Deactivate the relay
    print("Relay deactivated. Target height reached.")


# Main loop
def main():
    while not connect_wifi():
        print("Retrying WiFi connection in 5 seconds...")
        time.sleep(5)
    
    while not connect_mqtt():
        print("Retrying MQTT connection in 5 seconds...")
        time.sleep(5)

    while True:
        try:
            client.check_msg()  # Check for new MQTT messages
            
            # Optional: Publish current die height periodically
            success, die_height = readCurrentDieHeight()
            if success:
                msg = json.dumps({"height": die_height})
                client.publish(mqtt_topic, msg)
            else:
                print("Failed to read die height")
            
            time.sleep(1)
            
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


