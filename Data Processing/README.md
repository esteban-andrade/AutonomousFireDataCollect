# MATLAB Data Plotting Video Generator

This document outlines how to use the MATLAB script `data_plotter.m` to visualise time-based graphical data with a live video.

## Dependencies

This script was prepared using MATLAB R2021a. No specialised toolboxes are required for use.

## Purpose and Overview

The purspose of this program is to take a descrete time-based dataset in a `.csv` format and visualise that data against a video in realtime.

## Usage Instructions

To use this script, a csv and supported video file is required, and should be placed within your MATLAB workspace.

These filenames should be placed in lines 6 and 9:

```
% Load data to plot
sensorData = readtable('<csv_file>');

% Load video to plot
v = VideoReader('<video_file>');
```

Lines 1-3 should be updated to show the rate at which data is captured in the csv file, the frame rate of the video and the frame number within the video where the data starts recording:

```
data_poll_rate_hz = 1;
frame_rate_fps = 30;
start_frame = 222;
```

Lines 11-55 setup the interface, and may be modified should elements of the GUI require changing, such as aspect ratio, video size or number of graphics.

Lines 58-62 create the video object to output. Here, the file format, frame rate and compression ratio (quality) can be set.

If the original video needs to be rotated, this can be done on line 69:

```
frame = imrotate(frame, 270);
```

Lines 80-113 plot the figures, and needs to be configured depending on what data you wish to show, and how you wish to show it.

For example, line 81 can be configured to extract the appropriate data from the csv table, as well as format the plot.

## Disclaimer

It is acknowledged that the modularity of this script is limited, and can be improved in several ways. The user should understand that this script was developed as a "hacky prototype" used to visualise data for a specific application. This software is provided "as is" and is used at your own risk.