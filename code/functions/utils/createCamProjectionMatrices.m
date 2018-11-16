function [P1, P2] = createCamProjectionMatrices(cam_params)
%CREATECAMPROJECTIONMATRICES Creates projection matrices for a stereo
%camera. It is assumed that the parameters are for rectified images.
%
% Input:
%   - cam_params: structure comprising of fx, fy, cx, cy, baseline 
%
% Output:
%   - P1(3, 4): projection matrix for the left camera
%   - P2(3, 4): projection matrix for the right camera

% for left camera
P1 = [cam_params.fx, 0, cam_params.cx, 0; ...
      0, cam_params.fy, cam_params.cy, 0; ...
      0, 0, 1, 0];
% for right camera

P2 = [cam_params.fx, 0, cam_params.cx, - cam_params.base; ...
      0, cam_params.fy, cam_params.cy, 0; ...
      0, 0, 1, 0];
end

