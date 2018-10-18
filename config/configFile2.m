%%% Configuration File for visualSOFT
%%% Permits various adjustments to parameters of the Visual Odometry Algorithm

%% Read the calibration file to find parameters of the cameras 
A=[[7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00];
    [7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00]]; 
P1 = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
P2 = vertcat(A(2,1:4), A(2,5:8), A(2,9:12));

A = [[9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00]; ...
     [9.895267e+02 0.000000e+00 7.020000e+02 0.000000e+00 9.878386e+02 2.455590e+02 0.000000e+00 0.000000e+00 1.000000e+00]];
K1 = vertcat(A(1,1:3), A(1,4:6), A(1,7:9));
K2 = vertcat(A(2,1:3), A(2,4:6), A(2,7:9));

%% Paramters for Feature Selection using bucketing
bucketSize = 50; % Size of each bucket
numCorners = 2;  % Maximum Number of features to be selected from each bucket

%% Paramters for RANSAC algorithm to exclude outliers during rotation estimation 
s = 15; % smallest number of points required to fit the model
w = 0.9; % Percentage number of inliers desired
p = 0.9999; %Accuracy in fitted model desired