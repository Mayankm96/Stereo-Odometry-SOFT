function points3D = gen3dPoints(pts_l, pts_r, P1, P2)
%%GEN3DPOINTS Given matched points in the left and right frames of the
% stereo system along with the projection matrices for each camera, a 
% 3D point cloud is generated.
%
% INPUT:
%   - pts_l(N, 2): matched feature points locations in left camera frame
%   - pts_r(N, 2): matched feature points locations in right camera frame
%   - P1(3, 4): Projection matrix for 
%   - P2(3, 4): Projection matrices for left and right cameras respectively
%
% OUTPUT:
%   - points3D(3, N): 3-D locations of points in world coordinate system

% invert x-y corrdinates for image processing tooblox
im_pts1_l = [pts_l(:, 2), pts_l(:, 1)];
im_pts1_r = [pts_r(:, 2), pts_r(:, 1)];

points3D = triangulate(im_pts1_l, im_pts1_r, P1', P2')';

end

