import os
import subprocess
import time
import signal
import serial

portPath = "/dev/ttyACM0"       # Must match value shown on Arduino IDE
baud = 115200                     # Must match Arduino baud rate
timeout = 5                       # Seconds
READ_DELAY_SECONDS = 2

def create_serial_obj(portPath, baud_rate, tout):
    """
    Given the port path, baud rate, and timeout value, creates
    and returns a pyserial object.
    """
    return serial.Serial(portPath, baud_rate, timeout=tout)


def read_serial_data(serial):
    """
    Given a pyserial object (serial). Outputs a list of lines read in
    from the serial port
    """
    serial.flushInput()

    serial_data = []
   
  
    # serial_line = serial.readline()
     
    # serial_data.append(serial_line)
         

    # return serial_data

    bytesToRead = serial.inWaiting()
    serial.read(bytesToRead)

    return bytesToRead

def send_serial_data(serial,x):

    serial.write(bytes(x, 'utf-8'))
    time.sleep(0.5)
    return x


def main():

    visual = 'roslaunch flir_one_node flir_data_vis.launch'
    record = 'roslaunch flir_one_node flir_data_record.launch'

    serial_obj = create_serial_obj(portPath, baud, timeout)
    
    while True:

        is_recording =False
        serial_data = read_serial_data(serial_obj)

        if serial_data == '1':

            ############## Starts process######
            time.sleep(READ_DELAY_SECONDS)
            print("record")
            # proc = subprocess.Popen(
            #     [record], stdout=subprocess.PIPE,  shell=True, preexec_fn=os.setsid)
            # # send_serial_data(serial_obj,"x") # to change LEDs
            is_recording = True

            while(is_recording):
                serial_data = read_serial_data(serial_obj)

                if serial_data == '0':
                    time.sleep(READ_DELAY_SECONDS)
                    # os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    print("stop")
                    time.sleep(READ_DELAY_SECONDS)
                    # send_serial_data(serial_obj, "y") #to change LEDS
                    is_recording=False
                   
                      


######################
    # time.sleep(20)  # <-- sleep for 12''



###################terminates##########################
    # os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
   


  

if __name__ == "__main__":
    main()
