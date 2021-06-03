# Interaction Section

This component describes the integration between the user interface and the system.

## Requirements.
Ensure to navigate to the `test folder` and compile the test  script
```
mkdir build 
cd build 
cmake ..
make
````

## Specifications.
This will generate an executable(program) that will be added to the `start-up` applications in ubuntu in order to start it automatically after boot.

It will read data from the serial port `/dev/ttyACM0` amd it will wait from input from the user in order to start recording data.

Once the user pressed the stop button it will terminate the data recording process.