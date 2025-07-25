import math
import time
import serial

SERIAL_PORT = "COM3"  # Modify this to your serial port
BAUD_RATE = 115200

def send_phase(ser, phase):
    # Send the phase value as a string followed by newline
    ser.write(f"{phase}\n".encode())
    time.sleep(0.1)  # Short delay to allow Arduino to process
    # Read response from Arduino
    response = ser.readline().decode("utf-8").strip()
    print(f"Sent phase: {phase:.4f}, Arduino response: {response}")

if __name__ == "__main__":
    # List of phase values: 0, π/3, 2π/3
    phases = [0, math.pi / 3, 2 * math.pi / 3]

    # Open the serial port
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        # Loop forever, sending each phase in sequence
        while True:
            for phase in phases:
                send_phase(ser, phase)
                time.sleep(1)  # Wait 1 second before sending next phase
