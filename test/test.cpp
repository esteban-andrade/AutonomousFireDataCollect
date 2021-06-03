// #include <iostream>
// #include <stdio.h>
// #include <termios.h>
// #include <unistd.h>
// #include <string.h>
// #include <errno.h>
// #include <vector>

// using namespace std;

// int main(int argc, char ** argv)
// {
// 	char *levelSensorPort = "/dev/ttyUSB0"; //Serial Device Address

// 	int levelSensor = serialOpen(levelSensorPort, 19200);
// 	wiringPiSetup();
// 	serialPuts(levelSensor, "DP"); //Send command to the serial device

// 	while (1)
// 	{
// 		char buffer[100];
// 		ssize_t length = read(levelSensor, &buffer, sizeof(buffer));
// 		if (length == -1)
// 		{
// 			cerr << "Error reading from serial port" << endl;
// 			break;
// 		}
// 		else if (length == 0)
// 		{
// 			cerr << "No more data" << endl;
// 			break;
// 		}
// 		else
// 		{
// 			buffer[length] = '\0';
// 			cout << buffer; //Read serial data
// 		}
// 	}

// 	return 0;
// }

// C library headers
// #include <stdio.h>
// #include <string.h>

// // Linux headers
// #include <fcntl.h>   // Contains file controls like O_RDWR
// #include <errno.h>   // Error integer and strerror() function
// #include <termios.h> // Contains POSIX terminal control definitions
// #include <unistd.h>  // write(), read(), close()

// int main()
// {
//     int serial_port = open("/dev/ttyACM0", O_RDWR);

//     // Check for errors
//     if (serial_port < 0)
//     {
//         printf("Error %i from open: %s\n", errno, strerror(errno));
//     }

//     // Create new termios struct, we call it 'tty' for convention
//     // No need for "= {0}" at the end as we'll immediately write the existing
//     // config to this struct
//     struct termios tty;

//     // Read in existing settings, and handle any error
//     // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
//     // must have been initialized with a call to tcgetattr() overwise behaviour
//     // is undefined
//     if (tcgetattr(serial_port, &tty) != 0)
//     {
//         printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
//     }

//     cfsetispeed(&tty, B115200);
// }

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <iostream>
#include <unistd.h>

#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>

#include <signal.h>
#include <stdio.h>

int main()
{
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    // Write to serial port
    //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
    // write(serial_port, "Hello, world!", sizeof(msg));

    unsigned char msg[] = {'1'};
    write(serial_port, "1", sizeof(msg));
    bool state = false;
    pid_t pid;

    pid = fork();
    while (1)
    {
        // Allocate memory for read buffer, set size according to your needs
        // bool read_buf;

        bool read_buf;

        bool tracker;
        // Normally you wouldn't do this memset() call, but since we will just receive
        // ASCII data for this example, we'll set everything to 0 so we can
        // call printf() easily.
        memset(&read_buf, '\0', sizeof(read_buf));

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0)
        {
            printf("Error reading: %s", strerror(errno));
            return 1;
        }

        // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
        // print it to the screen like this!)
        //printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
        std::cout << read_buf << std::endl;
        if (read_buf == true)
        {
            state = true;
            tracker = true;
        }
        else if (read_buf == false)
        {
            tracker = false;
        }

        if (state == true && tracker == true)
        {

            system("gnome-terminal -- sh -c 'roslaunch flir_one_node flir_data_record.launch'");
            // system("roslaunch flir_one_node flir_data_record.launch");
            state = false;
        }
        else if (tracker == false && state == false)
        {
            // string s = "pkill -f " + filename "+";
            // sleep 2;
            // pkill - 9 - f "+filename;
            //     system(s.c_str());

            system("kill -9 $(ps ax | grep flir | fgrep -v grep | awk '{ print $1 }')");
        }
       
    }
    // write(serial_port, "0", sizeof(msg));
    close(serial_port);

    return 0; // success
}
