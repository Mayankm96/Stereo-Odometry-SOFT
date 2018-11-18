function [R, tr, vo_previous_updated] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous)
% VISUALSOFT Given the timestep t , Stereo Odometry based on careful Feature selection
% and Tracking is implemented to estimate the rotation and translation between
% the frames at t-1 and t
%
% INPUT:
%   - t: frame instant for which computation is being performed
%   - I1_l: left image for time t-1
%   - I2_l: left image for time t
%   - I1_r: right images for time t-1
%   - I2_r: right images for time t
%   - P1(3, 4): projection matrix of camera 1 (left)
%   - P2(3, 4): rojection matrix of camera 2 (right)
%   - vo_params: structure containing parameters for the algorithm
%   - vo_previous: structure containing certain data from time step t-1
%
% OUTPUT:
%   - R(3, 3): Rotation estimate between time t-1 and t
%   - tr(1, 3): Translation estimate between time t-1 and t
%   - vo_previous_updated: structure containing certain data from time step t

%% Initialize parameters
K1 = P1(1:3, 1:3);      % intrinsic calibration matrix for left camera
dims = size(I2_l);      % dimensions of image (height x width)
time = zeros(4, 1);      % variable to store time taken by each step

%% Feature points extraction
tic;
% compute new features for current frames
pts2_l = computeFeatures(I2_l, vo_params.feature);
pts2_r = computeFeatures(I2_r, vo_params.feature);
% retrieve extracted features from time t-1
pts1_l = vo_previous.pts1_l;
pts1_r = vo_previous.pts1_r;
time(1) = toc;

%% Circular feature matching
tic;
matches = matchFeaturePoints(I1_l, I1_r, pts1_l, pts2_l, pts1_r, pts2_r, dims, vo_params.matcher);
time(2) = toc;

%% Feature Selection using bucketing
tic;
well_matches = bucketFeatures(matches, vo_params.bucketing);
time(3) = toc;

%% Rotation (R) and Translation(tr) Estimation by minimizing Reprojection Error
m_pts1_l = horzcat(well_matches(:).pt1_l);  location1_l = vertcat(m_pts1_l(:).location);
m_pts1_r = horzcat(well_matches(:).pt1_r);  location1_r = vertcat(m_pts1_r(:).location);
m_pts2_l = horzcat(well_matches(:).pt2_l);  location2_l = vertcat(m_pts2_l(:).location);

% 3D Point Cloud generation at time t-1 using Triangulation
tic;
points3D_1 = gen3dPoints(location1_l, location1_r, P1, P2);

% motion estimation by minimizing reprojection error and RANSAC

% invert x-y corrdinates for image processing tooblox
location2_l = [location2_l(:, 2), location2_l(:, 1)];
cam1 = cameraIntrinsics([K1(1, 1), K1(2,2)], [K1(1, 3), K1(2, 3)], dims);
[R, tr] = estimateWorldCameraPose(location2_l, points3D_1', cam1);

time(4) = toc;
%% plotting

fprintf('Time taken for feature processing: %6.4f\n', time(1));
fprintf('Time taken for feature matching: %6.4f\n', time(2));
fprintf('Time taken for feature selection: %6.4f\n', time(3));
fprintf('Time taken for motion estimation: %6.4f\n', time(4));

figure(1);
subplot(2, 2, 1);
showFlowMatches(I1_l, I2_l, matches, '-y', 1, '+', 2);
title(sprintf('Flow Matched Features in left camera at frame %d', t));

% show features before bucketing
figure(1);
subplot(2, 2, 3);
imshow(I2_l);
hold on;
m_pts2_l = horzcat(matches(:).pt2_l);
plotFeatures(m_pts2_l,  '+r', 2, 0)
% show features after bucketing
m_pts2_l = horzcat(well_matches(:).pt2_l);
plotFeatures(m_pts2_l,  '+g', 2, 0)
title(sprintf('Feature selection using bucketing at frame %d', t))

%% Preparation for next iteration
% allocate features detected in current frames as previous
vo_previous_updated.pts1_l = pts2_l;
vo_previous_updated.pts1_r = pts2_r;

end
