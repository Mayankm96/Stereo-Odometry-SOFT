clc
clear
close all
addpath('../config');

%% Execute the configuration file to read parameters for data paths
configFile1;

%% Initialize variables for pose of the system
pos = [0;0;0];
Rpos = eye(3);

%% Start Algorithm
for i = 1: NumDataSet
    %% Implement SOFT for time instant i. Previous frame at time instant t-1
    tic;
    [R, t] = visualSOFT(i, path1, path2, style);
    toc
    
    %% Plot the odometry transformed data
    pos = pos + Rpos*t;
    Rpos = R*Rpos;
    plot(pos(1),pos(3),'ob');
    hold on;
    pause(0.005);
end
toc