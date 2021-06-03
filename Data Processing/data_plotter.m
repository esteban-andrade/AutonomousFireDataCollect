data_poll_rate_hz = 1;
frame_rate_fps = 30;
start_frame = 222;

% Load data to plot
sensorData = readtable('2021-04-27 21-51-27 SENSOR DATA.csv');

% Load video to plot
v = VideoReader('20210428_123544000_Converted.mp4');

% Construct GUI
padding_graph_x = 0.07;
padding_graph_y = 0.1;
padding_image = 0.02;
frame_ratio = 1280/720;

f = figure;
f.Position = [10, 10, 1280, 720];

image_height_ratio = (1 - 2*padding_image);
image_width_ratio = image_height_ratio/frame_ratio^2;
posImage = [padding_image padding_image image_width_ratio image_height_ratio];
image_plot = subplot('Position',posImage);

x1 = image_width_ratio + padding_graph_x + padding_image;
y1 = padding_graph_y;

height = (1 - 3*padding_graph_y) / 2;
width = (1 - x1 - 2*padding_graph_x) / 2;

x2 = x1 + width + padding_graph_x;
y2 = 2*padding_graph_y + height;

posGraph1 = [x1 y1 width height];
posGraph2 = [x1 y2 width height];
posGraph3 = [x2 y1 width height];
posGraph4 = [x2 y2 width height];

plot_time = max(sensorData.SecondsElapsed);

plot1 = subplot('Position',posGraph1);
max_temp_plot = ceil(max(sensorData.Temp)/5)*5;
min_temp_plot = floor(min(sensorData.Temp)/5)*5;

plot2 = subplot('Position',posGraph2);
max_mq2_plot = ceil(max(sensorData.MQ2_Gas)/5)*5;
min_mq2_plot = floor(min(sensorData.MQ2_Gas)/5)*5;

plot3 = subplot('Position',posGraph3);
max_bme_gas_plot = ceil(max(sensorData.BME680_Gas)/5)*5;
min_bme_gas_plot = floor(min(sensorData.BME680_Gas)/5)*5;

plot4 = subplot('Position',posGraph4);
max_pcpm_plot = ceil(max(max([sensorData.PC2_5, sensorData.PM2_5]))/5)*5;
min_pcpm_plot = 0;


% Create video object
vobj = VideoWriter('data_plot','Motion JPEG AVI');
vobj.FrameRate = frame_rate_fps;
vobj.Quality=60;
open(vobj); 

hold on

% Loop through each frame in the video
for frame_index = 1:v.NumFrames
    frame = read(v, frame_index); % Get frame from video
    frame = imrotate(frame, 270); % Rotate frame
    imshow(frame, 'Parent', image_plot)
    recording_data_frame = frame_index - start_frame;
    if (recording_data_frame > 0)
        if mod(recording_data_frame, frame_rate_fps)
            data_index = floor(recording_data_frame / frame_rate_fps) + 1;
            xaxis_lim = ceil(sensorData.SecondsElapsed(data_index) * (4/3));
            if (xaxis_lim < 10)
                xaxis_lim = 10;
            end

            % Plot Temperature
            plot(sensorData.SecondsElapsed(1:data_index), sensorData.Temp(1:data_index), "r-*", 'Parent', plot1);
            xlim(plot1, [0 xaxis_lim])
            ylim(plot1, [min_temp_plot max_temp_plot])
            title(plot1, "Temperature");
            xlabel(plot1, "Time (s)");
            ylabel(plot1, ['Temperature ' char(176) 'C'])

            % Plot MQ2 Gas
            plot(sensorData.SecondsElapsed(1:data_index), sensorData.MQ2_Gas(1:data_index), "b-*", 'Parent', plot2);
            xlim(plot2, [0 xaxis_lim])
            ylim(plot2, [min_mq2_plot max_mq2_plot])
            title(plot2, "MQ2-Gas");
            xlabel(plot2, "Time (s)");
            ylabel(plot2, "Sensor Reading");

            % Plot BME_680 Gas
            plot(sensorData.SecondsElapsed(1:data_index), sensorData.BME680_Gas(1:data_index), "b-*", 'Parent', plot4);
            xlim(plot4, [0 xaxis_lim])
            ylim(plot4, [min_bme_gas_plot max_bme_gas_plot])
            title(plot4, "BME680 Gas");
            xlabel(plot4, "Time (s)");
            ylabel(plot4, "Sensor Reading");

            % Plot PC and PM
            hold on
            plot(sensorData.SecondsElapsed(1:data_index), sensorData.PM2_5(1:data_index), "g-*", 'Parent', plot3);
            hold on
            plot(sensorData.SecondsElapsed(1:data_index), sensorData.PC2_5(1:data_index), "m-*", 'Parent', plot3);
            xlim(plot3, [0 xaxis_lim])
            ylim(plot3, [min_pcpm_plot max_pcpm_plot])
            title(plot3, "PM2.5 (GREEN) and PC2.5 (MAGENTA)");
            xlabel(plot3, "Time (s)");
            ylabel(plot3, "Sensor Reading");
        end
    end
    writeVideo(vobj, getframe(f));
end
close(vobj);
close(gcf)