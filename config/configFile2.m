%%% Configuration File for visualSOFT
%%% Permits various adjustments to parameters of the Visual Odometry Algorithm

%% Paramters for Feature Selection using bucketing
bucketSize = 50; % Size of each bucket
numCorners = 2;  % Maximum Number of features to be selected from each bucket

%% Paramters for RANSAC algorithm to exclude outliers during rotation estimation
s = 15; % smallest number of points required to fit the model
w = 0.9; % Percentage number of inliers desired
p = 0.9999; %Accuracy in fitted model desired
