from umodbus.serial import Serial
from machine import Pin
import time

# Define the pins for Modbus communication
rtu_pins = (Pin(17), Pin(18))

# Initialize the Modbus object
m = Serial(baudrate=9600, data_bits=8, stop_bits=1, parity=None, pins=rtu_pins, ctrl_pin=15, uart_id=2)

# Increase timeout for Modbus response
m._uart.init(timeout=2000)  # Set UART timeout to 2000 ms (2 seconds)

# Function to read current die height
def readCurrentDieHeight():
    slave_addr = 0x08  # Modbus slave address (updated to 8)
    starting_address = 0x00  # Start reading from address 0x0000
    register_quantity = 2  # Read two registers (32-bit value)

    # Try reading the holding registers
    try:
        time.sleep(0.2)  # Small delay to give slave time to respond
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

        print(f"Current Die Height: {die_height:.4f}")
        return True, die_height

    except Exception as e:
        print(f"Error reading Modbus registers: {e}")
        return False, 0.0

# Continuous reading loop
while True:
    success, die_height = readCurrentDieHeight()
    if success:
        print(f"Die Height: {die_height:.4f}")
    else:
        print("Error reading die height")
    
    time.sleep(1)

