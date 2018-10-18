function [R, tr] = visualSOFT(t, path1, path2, style)
% Given the timestep t , Stereo Odometry based on careful Feature selection 
% and Tracking is implemented to estimate the rotation and translation between 
% the frames at t-1 and t 
%   t: time instant at which current frame received
%   path1, path2: Path to directories where the dataset is stored
%   style: style of the name of images stored (first image name)

addpath('functions');
addpath('../config');

%% Execute the configuration file to read parameters for SOFT Algorithm
configFile2;

%% Read images
dig1 = imval2str(style,t-1);
dig2 = imval2str(style,t);

%% Read Images with names dig1 and dig2
I1_l = imread(strcat(path1,dig1,'.png'));
I1_r = imread(strcat(path2,dig1,'.png'));
I2_l = imread(strcat(path1,dig2,'.png'));
I2_r = imread(strcat(path2,dig2,'.png'));

%% Feature Tracking at time instant t
% In our implementation we have used the  Kanade-Lucas-Tomasi (KLT) algorithm
% to track the features in the left camera at time instant t

pts1_l = detectMinEigenFeatures(I1_l,'FilterSize',5,'MinQuality',0);
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, pts1_l.Location, I1_l);
[pts2_l, validity] = step(tracker, I2_l);

pts1_l(validity(:)==0,:) = [];
pts2_l(validity(:)==0,:) = [];
pts2_l=cornerPoints(pts2_l);

%% Feature points extraction
% In the original paper, filter masks followed by Non- Maximal Suppresion are
% used to detect the features. However, since the MinEigen algorithm works well
% for feature detection we have used its MATLAB implementation directly.

pts1_r = detectMinEigenFeatures(I1_r,'FilterSize',5,'MinQuality',0);
pts2_r = detectMinEigenFeatures(I2_r,'FilterSize',5,'MinQuality',0);

%% Circular matching
% In the original paper, sparse SSD circular matching has been done.
% However, to simplify the implementation of visual odometry we have
% done circular matching directly using the matchFeatures function in 
% MATLAB over Binary Features. 

% compare left frame at t with left frame at t-1
[pts2_l, pts1_l] = matchFeaturePoints(I2_l, I1_l, pts2_l, pts1_l);

% compare left frame at t-1 with right frame at t-1
[pts1_l, pts1_r] = matchFeaturePoints(I1_l, I1_r, pts1_l, pts1_r);

% compare right frame at t-1 with right frame at t
[pts1_r, pts2_r] = matchFeaturePoints(I1_r, I2_r, pts1_r, pts2_r);

% compare right frame at t with left frame at t
[pts2_r, pts2_l] = matchFeaturePoints(I2_r, I2_l, pts2_r, pts2_l);

%% Feature Selection using bucketing
subplot(3,1,1);
imshow(I2_l);
title(sprintf('Feature selection using bucketing at frame %d', t))
hold on
scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+r');
pts2_l = bucketFeatures(I2_l, pts2_l, bucketSize, numCorners);
scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+g');

%% Rotation(R) Estimation using Nister's Algorithm

% Feature Matching to get corresponding points at time instant t and t-1 in
% the left and right cameras
[pts2_l, pts1_l] = matchFeaturePoints(I2_l, I1_l, pts2_l, pts1_l);

subplot(3,1,2);
showMatchedFeatures(I2_l, I1_l, pts2_l, pts1_l);
title(sprintf('Matched Features in left camera at frame %d', t))

% RANSAC algorithm to exclude outliers and estimate rotation matrix
R = estimRotation(pts1_l, pts2_l, K1, K1, s, w, p);

%% Translation(tr) Estimation by minimizing Reprojection Error
[pts2_l, pts2_r] = matchFeaturePoints(I2_l, I2_r, pts2_l, pts2_r);
[pts1_l, pts2_l] = matchFeaturePoints(I1_l, I2_l, pts1_l, pts2_l);

% 3D Point Cloud generation at time t-1 using Triangulation
[pts1_l, pts1_r] = matchFeaturePoints(I1_l, I1_r, pts1_l, pts1_r);
points3D_1 = gen3dPoints(pts1_l, pts1_r, P1, P2);

% Minimization of cost function to find translation
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',1500);

tr0 = [0; 0; 0];
errorCostFunction = @(tr)double(reprojectenError([tr(1); tr(2); tr(3)], R, K1, K2, points3D_1, pts2_l, pts2_r));
tr = lsqnonlin(errorCostFunction, tr0, [], [], options);

end