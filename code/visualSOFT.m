function [R, tr] = visualSOFT(t, I1_l , I1_r, I2_l, I2_r, P1, P2, vo_params)
% VISUALSOFT Given the timestep t , Stereo Odometry based on careful Feature selection
% and Tracking is implemented to estimate the rotation and translation between
% the frames at t-1 and t
%
% INPUT:
%   - t: frame instant for which computation is being performed
%   - I1_l, I1_r: stereo pair of left and right images for time t-1
%   - I2_l, I2_r: stereo pair of left and right images for time t
%   - P1(3, 4): projection matrix of camera 1 (left)
%   - P2(3, 4): rojection matrix of camera 2 (right)
%   - vo_params: structure containing parameters for the algorithm
%
% OUTPUT:
%   - R(3, 3): Rotation estimate between time t-1 and t
%   - R(3, 3): Translation estimate between time t-1 and t

%% Intrinsic camera matrices for rectified images
K1 = P1(1:3, 1:3);
K2 = P2(1:3, 1:3);
cam1 = cameraIntrinsics([K1(1, 1), K1(2,2)], [K1(1, 3), K1(2, 3)], size(I1_l));
cam2 = cameraIntrinsics([K2(1, 1), K2(2,2)], [K2(1, 3), K2(2, 3)], size(I2_l));

%% Feature points extraction
% In the original paper, filter masks followed by Non- Maximal Suppresion are
% used to detect the features. However, since the FAST algorithm works well
% for feature detection we have used its MATLAB implementation directly.

pts1_r = detectFASTFeatures(I1_r);
pts1_l = detectFASTFeatures(I1_l);
pts2_l = detectFASTFeatures(I2_l);
pts2_r = detectFASTFeatures(I2_r);

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
[~, pts2_l] = matchFeaturePoints(I2_r, I2_l, pts2_r, pts2_l);

%% Feature Selection using bucketing
subplot(2,2,1);
imshow(I2_l);
title(sprintf('Feature selection using bucketing at frame %d', t))
hold on
scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+r');
pts2_l = bucketFeatures(I2_l, pts2_l, vo_params.bucketing.bucket_width, vo_params.bucketing.max_features);
scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+g');

%% Feature Matching to get corresponding points at time instant t and t-1 in
% the left and right cameras
[pts2_l, pts1_l] = matchFeaturePoints(I2_l, I1_l, pts2_l, pts1_l);

subplot(2,2,3);
showMatchedFeatures(I2_l, I1_l, pts2_l, pts1_l);
title(sprintf('Matched Features in left camera at frame %d', t))

%% Rotation (R) and Translation(tr) Estimation by minimizing Reprojection Error

% % 3D Point Cloud generation at time t-1 using Triangulation
[pts1_l, pts1_r] = matchFeaturePoints(I1_l, I1_r, pts1_l, pts1_r);
points3D_1 = gen3dPoints(pts1_l, pts1_r, P1, P2);

if pts2_l.Count < 4
    R = eye(3);
    tr = zeros(1, 3);
else
    [R,tr] = estimateWorldCameraPose(pts2_l.Location, points3D_1', cam1);
end
end
