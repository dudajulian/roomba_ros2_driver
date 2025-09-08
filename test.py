import serial
import time

# ser = serial.Serial('/dev/ttyUSB2', 57600)

# Configure serial connection
ser = serial.Serial(
    port="/dev/ttyUSB0",      
    baudrate=57600,
    timeout=0.1
)

# Check if port is open
while not ser.is_open:
    time.sleep(0.2)

print("Serial connection established")
time.sleep(5)

# Initialize
print("Initializing...")
ser.write(bytes([128]))  # Start
time.sleep(2.5)
# ser.write(bytes([130]))  # Control
# time.sleep(5)
ser.write(bytes([132]))  # Full mode
time.sleep(2.5)

# Drive forward 200 mm/s, straight
print("Driving forward...")
# ser.write(bytes([137, 0x00, 0xC8, 0x80, 0x00]))
ser.write(bytes([137, 0x00, 0xC8, 0x00, 0xC8]))  # both wheels 200 mm/s
time.sleep(10)

# Stop
print("Stopping...")
ser.write(bytes([137, 0x00, 0x00, 0x00, 0x00]))

print("Commands sent successfully.")
