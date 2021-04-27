import time
import datetime
import sys
import readkey

from serial_list import serial_ports
from serial_handler import SerialHandler, SerialException

READ_DELAY_SECONDS = 0.5
BAUD_RATE = 115200

def getSerialPort():
    serial_port_list = serial_ports()
    valid_serial_port = None
    for serial_port in serial_port_list:
        # Connect to serial port
        serial_device = SerialHandler()
        serial_device.start(serial_port, BAUD_RATE, timeout_seconds = 1)

        # Stop recording if MCU is recording
        serial_device.send_string('0')
        time.sleep(READ_DELAY_SECONDS)

        # Send call to verify connection
        serial_device.send_string('J')
        time.sleep(READ_DELAY_SECONDS)
        response = serial_device.pop_bytes_all()

        # Check if response is valid
        if response == b'I':
            valid_serial_port = serial_port

        # Close connection
        del serial_device

        # If a valid port was found, break
        if valid_serial_port:
            break
        
    return valid_serial_port

def main():
    while True:
        is_recording = False
        # Wait for R to be pressed
        print("Press R to start recording or Q to quit...")
        key = readkey.readkey()
        if key == 'r':
            print("Recording...")
            is_recording = True
        if key == 'q':
            break
        if is_recording:
            valid = False
            device = None
            try:
                port = getSerialPort()
            except Exception as e:
                print(f"FATAL ERROR {e}")
                sys.exit(-1)
            valid = True
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H-%M-%S")
            if port is not None:
                try:
                    print(f"Connecting to device on {port} at baud: {BAUD_RATE}")
                    device = SerialHandler()
                    device.start(port, BAUD_RATE, timeout_seconds = 1, log_filename = f"{timestamp} SENSOR DATA", log_extension = "csv", log_show_timestamp = True)
                except SerialException as e:
                    print(f"CONNECTION ERROR {e}")
                    port = None
                    valid = False
                    if device:
                        del device
                    device = None
            else:
                valid = False
            if valid:
                if device:
                    try:
                        device.send_string("1")
                    except SerialException as e:
                        print(f"Error initialising data collection {e}")
                while (is_recording):
                    # Wait for S to be pressed
                    key = readkey.readkey()
                    if key == 's':
                        time.sleep(READ_DELAY_SECONDS)
                        is_recording = False
                        if device:
                            if device:
                                try:
                                    device.send_string("0")
                                except SerialException as e:
                                    print(f"Error stopping data collection {e}")
                            del device


if __name__ == '__main__':
    main()