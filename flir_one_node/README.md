# FLIR One Interfacing Node

This package is used to open a connection to the FLIR one camera and publish data from it to various ROS topics.

This guide assumes that [ROS](https://www.ros.org/) has been installed and setup on the user's system

Package has been tested using `ROS Melodic` running on `Ubuntu 18.04`

## 1. Allow access to USB device

The FLIR One has a vendor ID of `0x09CB` and a porduct ID of `0x1996`

To allow the package to access data from this, create a new `rules` file

```
sudo nano /etc/udev/rules.d/51-usb-flir-one.rules
```

Add the following text to this file

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="09cb", ATTRS{idProduct}=="1996", MODE:="0666"
```

Restart the system

## 2. Create and run the ROS package

Ensure that this git repository is contained within the `src` folder of the `catkin_ws` directory.

Run `catkin_make` from within the `catkin_ws` directory. Otherwise perform a `catkin build`

Ensure the package can be run by executing the following command from the `catkin_ws` directory: 

```
source devel/setup.bash
```

If there is an issue once when running the program starts on the computer unit. Refresh the bashrc.
```
source devel/setup.bash
```




Ensure an instance of ROS is running on the machine

```
roscore
```

Ensuring the camera is connected and powered on, run the ROS package

```
rosrun flir_one_node flir_one_node
```
## Launch Files

Alternatively there are a series of launch files that can be used to start the rosnode. These will allow a simpler approach when executing the ROS nodes for different purposes such as Visualization or Data Recording

### For Visualization 

Type the following command in a terminal. It will start the node and it will broadcast both RBG and Depth Images

```
roslaunch flir_one_node flir_data_vis.launch
```

### For Recording
 
 Type the following command in the terminal to record a ROSBAG of images and sensor data. The ROSBAG will use the timestamp of when the recording process started.
 
```
roslaunch flir_one_node flir_data_vis.launch
```


The data can be viewed using RQT (or similar) on the following topics:

```
/camera_flir_node/rgb/image_raw
/camera_flir_node/ir_8b/image_raw
/camera_flir_node/ir_16b/image_raw
/flirone/ok/status 
/sensors/data
```
The `/flirone/ok/status ` will broadcast the status of the connection of the Flir camera with the Intel Nuc. A boolean will be used and if the value is true there are no issues when using camera to record data.

If the Visualization Launch file is triggered it will use RQT as default for image visualization


## SensorData

The sensor data will be parsed using strings. 
Ensure that the microcontroller is defined as `/dev/ttyACM0`


The Node will send serial data to the microcontroller when it  is required to start and stop the sensor data acquisition process.

he topic `/sensors/data` will receive the sensor data from the teensy and it will parse the serial data as string and subsequenly advertised in this ROS topic.


