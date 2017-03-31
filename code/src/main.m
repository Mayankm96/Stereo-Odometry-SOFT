clc
clear
close all
addpath('functions')

%% Execute the configuration file to read parameters
configFile;

%% Initialize variables for pose of the system
pos = [0;0;0];
Rpos = eye(3);

%% START ALGORITHM
tic;
for i = 0: (NumDataSet-1)
    %% Read images
    dig1 = imval2str(style,i);
    dig2 = imval2str(style,i+1);

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
    
    %figure; imshow(I2_l);
    %hold on
    %scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+r');
    pts2_l = bucketFeatures(I2_l,pts2_l,bucketSize,numCorners);
    %scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+g');

    %% Rotation Estimation using Nister's Algorithm

    % Feature Matching to get corresponding points at time instant t and t-1 in
    % the left camera
    [pts2_l, pts1_l]=matchFeaturePoints(I2_l, I1_l, pts2_l, pts1_l);

    %figure; showMatchedFeatures(I2_l, I1_l, pts2_l, pts1_l);

    % RANSAC algorithm to exclude outliers and estimate rotation matrix
    R = estimRotation(pts1_l, pts2_l, K1, K1, s, w, p );
    
    %% Translation Estimation by minimizing Reprojection Error
    %%3D Point Cloud generation at time t-1 using Triangulation
    [pts1_l, pts1_r] = matchFeaturePoints(I1_l, I1_r, pts1_l, pts1_r);
    %points3D_1 = gen3dPoints(pts1_l, pts1_r, P1, P2);
    
    %%3D Point Cloud generation at time t-1 using Triangulation
    [pts2_l, pts2_r] = matchFeaturePoints(I2_l, I2_r, pts2_l, pts2_r);
    points3D_2 = gen3dPoints(pts2_l, pts2_r, P1, P2)';
    
    t=zeros(3,1);
    reprojError=@(t)reprojectenError(R, t, K1, K2, points3D_2, pts2_l, pts2_r);
    t=fminunc(reprojError,t);

    %% Plot the odometry transformed data
    pos = pos + Rpos*t;
    Rpos = R*Rpos;
    plot(pos(1),pos(3),'ob');
    hold on;
    pause(0.005);
end
toc