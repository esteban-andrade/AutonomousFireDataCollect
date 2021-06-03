import serial

# # Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2,
#                     xonxoff=False, rtscts=False, dsrdtr=False)
# ser.flushInput()
# ser.flushOutput()


# while True:
#   bytesToRead = ser.inWaiting()
#   ser.read(bytesToRead)

#   print(bytesToRead)

# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = '/dev/ttyACM0'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)
    while True:
        # using ser.readline() assumes each line contains a single reading
        # sent using Serial.println() on the Arduino
        # reading = ser.readline().decode('utf-8')
        # reading is a string...do whatever you want from here
        bytesToRead = ser.inWaiting()
        ser.read(bytesToRead)
        print(bytesToRead)


if __name__ == "__main__":
    main()
