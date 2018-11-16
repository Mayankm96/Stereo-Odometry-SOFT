function matches = matchFeaturePoints(pts1_l, pts2_l, pts1_r, pts2_r, dims, match_params)
%%MATCHFEATUREPOINTS Given feature points in two images, the function returns a
% set of points that have ben matched using SAD algorithm
%
% INPUT:
%   - pts1_l(1, N1): feature keypoints in images I1_l
%   - pts2_l(1, N2): feature keypoints in images I2_l
%   - pts1_r(1, N3): feature keypoints in images I1_r
%   - pts2_r(1, N4): feature keypoints in images I2_r
%   - dims(1, 2): shape of image (height x width)
%   - match_params: structure comprising of following
%       - match_binsize: matching bin width/height (for computation efficiency)
%       - match_radius: matching radius (dx/dy in pixels)
%       - match_disp_tolerance: dv tolerance for stereo matches (in pixels)
%       - match_uniqueness: ratio between best and second best match
%       - refinement: pixel location refinement (0=none,1=pixel,2=subpixel)
%
% OUTPUT:
%   - matches: array of structure of matched points across four images
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t

% perform circular matching
matches = performCircularMatching(pts1_l, pts2_l, pts1_r, pts2_r, dims, match_params);

% perform refinement

% remove outliers using RANSAC

end
