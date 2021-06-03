# AutonomousFireDataCollect

This repository deals with the Data Monitoring & Collection for Early-Stage Bushfires project for MMD AUT2021 held at UTS.

In recent years, there has been an international increase in the number, severity, and impact of bushfires on rural communities, biodiversity and subsequently both national and international economies. Currently, emergency services have no method to detect or collect data to monitor the early stages of remote bushfires. 


Currently, emergency services have limited methods to collect data to detect or monitor remote bushfires in their early stages. The first step is to build up a dataset of bushfire information from a range of sensors. A basic user-responsive prototype has been produced to collect and monitor bushfire data in low intensity areas which has been showcased here.


<img src="https://i.ibb.co/n8CS2R5/amp.jpg" width="400">

The following folders contain a solution that aids to capture data in order to train som ML models.

The main objective is to capture data using a thermal camera(Flir) along with auxiliary sensors(Temperature,Humidity,Pressure,etc.).
This will be processed using a corresponding ROS NODE ` flir_one_node` and with the Launch files start the data acquisition process smoothly and store it in a ROSBAG with the timestamp.

All Main components of this project are detailed in the folder below

* The Data Processing folder contains the script required to plot data in MATLAB from a video source and data set.
* The Electronics folder contains the code and PCB schematic/design files created for the prototype device.
* The Interaction folder contains the scripts required to accept commands from the Teensy device to the computer to trigger start and stop recording.
* The flir_one_node folder contains the files required to setup the FLIR ONE camera.
