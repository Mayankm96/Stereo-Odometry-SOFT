function [R, tr] = visualSOFT(t, I1_l , I1_r, I2_l, I2_r, P1, P2)
% Given the timestep t , Stereo Odometry based on careful Feature selection
% and Tracking is implemented to estimate the rotation and translation between
% the frames at t-1 and t
%   t: frame instant for which computation is being performed
%   I1_l, I1_r: stereo pair of left and right images for time t-1
%   I2_l, I2_r: stereo pair of left and right images for time t
%   P1(3, 4): projection matrix of camera 1 (left)
%   P2(3, 4): rojection matrix of camera 2 (right)

addpath(genpath('functions'));
addpath('config');

%% Execute the configuration file to read parameters for SOFT Algorithm
configFile2;

%% Intrinsic camera matrices for rectified images
K1 = P1(1:3, 1:3);
K2 = P2(1:3, 1:3);

%% Feature Tracking at time instant t
% In our implementation we have used the  Kanade-Lucas-Tomasi (KLT) algorithm
% to track the features in the left camera at time instant t

pts1_l = detectMinEigenFeatures(I1_l,'FilterSize',5,'MinQuality',0);
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, pts1_l.Location, I1_l);
[pts2_l, validity] = step(tracker, I2_l);

pts1_l(validity(:) == 0,:) = [];
pts2_l(validity(:) == 0,:) = [];
pts2_l = cornerPoints(pts2_l);

%% Feature points extraction
% In the original paper, filter masks followed by Non- Maximal Suppresion are
% used to detect the features. However, since the MinEigen algorithm works well
% for feature detection we have used its MATLAB implementation directly.

pts1_r = detectMinEigenFeatures(I1_r, 'FilterSize', 5, 'MinQuality', 0);
pts2_r = detectMinEigenFeatures(I2_r, 'FilterSize', 5, 'MinQuality', 0);

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
subplot(2,2,1);
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

subplot(2, 2, 3);
showMatchedFeatures(I2_l, I1_l, pts2_l, pts1_l);
title(sprintf('Matched Features in left camera at frame %d', t))

% RANSAC algorithm to exclude outliers and estimate rotation matrix
[R, tr] = estimRotationAndTranslation(pts1_l, pts2_l, K1, K1, s, w, p);

end
