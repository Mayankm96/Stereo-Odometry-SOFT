function cleaned_matches = removeOutliers(I1_l, I1_r, I2_l, matches, match_params)
%REMOVEOUTLIERS Performs outliers removal by applying two different approaches
% sequentially. The first method uses the normalized cross-correlation
% score between patches around the matched points. The second method uses
% RANSAC (or MSAC) to further cleanse the matches by estimating affine
% transformation between the matches.
%
% INPUT:
%
%   - I1_l: left image for time t-1
%   - I1_r: right images for time t-1
%   - I2_l: left image for time t
%   - matches (1,N): strcutre of matched points
%       - pt1_l: keypoint in left image at time t-1
%       - pt1_r: keypoint in right image at time t-1
%       - pt2_l: keypoint in left image at time t
%       - pt2_r: keypoint in right image at time t
%   - match_params: structure comprising of following
%       - match_ncc_window: window size of the patch for normalized cross-correlation
%       - match_ncc_tolerance: threshold for normalized cross-correlation
%       - ransac_iters: number of iteratios for RANSAC
%       - outlier_disp_tolerance: disparity tolerance (in pixels)
%       - outlier_flow_tolerance: flow tolerance (in pixels)
%
% OUTPUT:
%   - cleaned_matches(1, N2): processed structure of matched points
%       - pt1_l: location of keypoint in left image at time t-1
%       - pt1_r: location of keypoint in right image at time t-1
%       - pt2_l: location of keypoint in left image at time t
%       - pt2_r: location of keypoint in right image at time t

% initialize parameters
ncc_window = match_params.match_ncc_window;
ncc_tolerance = match_params.match_ncc_tolerance;
ransac_iters = match_params.ransac_iters;
outlier_disp_tolerance = match_params.outlier_disp_tolerance;


%% Step 1: Using Normalized Cross Correlation (NCC)

% for stereo --------------------------------------------------------------------

% store indices of matches that are valid
valid_match_indices = ones(length(matches), 1, 'logical');

% for all matches do
parfor i = 1:length(matches)
    match = matches(i);
    % verify match by computing NCC score
    validity = verifyMatchNCC(I1_l, I1_r, match.pt1_l.location, match.pt1_r.location, ...
                              ncc_window, ncc_tolerance);
    % if not valid, remove the match
    if not(validity)
        valid_match_indices(i) = 0;
    end
end

% cleaned matched points after NCC
matches = matches(valid_match_indices);

% for flow --------------------------------------------------------------------
% store indices of matches that are valid
valid_match_indices = ones(length(matches), 1, 'logical');

% for all matches do
parfor i = 1:length(matches)
    match = matches(i);
    % verify match by computing NCC score
    validity = verifyMatchNCC(I1_l, I2_l, match.pt1_l.location, match.pt2_l.location, ...
                              ncc_window, ncc_tolerance);
    % if not valid, remove the match
    if not(validity)
        valid_match_indices(i) = 0;
    end
end

% cleaned matched points after NCC
matches = matches(valid_match_indices);

%% Step 2: Using M-estimator SAmple Consensus (MSAC) algorithm

% for stereo --------------------------------------------------------------------

% store positions of matches at time t-1
m_pts1_l = vertcat(matches(:).pt1_l);
m_pts1_r = vertcat(matches(:).pt1_r);
location1_l = vertcat(m_pts1_l(:).location);
location1_r = vertcat(m_pts1_r(:).location);

% invert x-y corrdinates for image processing tooblox
im_location1_l = [location1_l(:, 2), location1_l(:, 1)];
im_location1_r = [location1_r(:, 2), location1_r(:, 1)];

% use MATLAB function to find inliers using MSAC
[~, inliers1_l, ~] = estimateGeometricTransform(im_location1_l, im_location1_r, 'affine', ...
                                                'MaxDistance', outlier_disp_tolerance, ...
                                                'MaxNumTrials', ransac_iters);

% find indices of valid matches
[~, valid_match_indices] = ismember(inliers1_l, im_location1_l, 'rows');

% cleaned matched points after MSAC
matches = matches(valid_match_indices);

% for flow --------------------------------------------------------------------

m_pts1_l = vertcat(matches(:).pt1_l);
m_pts2_l = vertcat(matches(:).pt2_l);
location1_l = vertcat(m_pts1_l(:).location);
location2_l = vertcat(m_pts2_l(:).location);

% invert x-y corrdinates for image processing tooblox
im_location1_l = [location1_l(:, 2), location2_l(:, 1)];
im_location2_l = [location2_l(:, 2), location2_l(:, 1)];

% use MATLAB function to find inliers using MSAC
[~, inliers1_l, ~] = estimateGeometricTransform(im_location1_l, im_location2_l, 'affine', ...
                                                'MaxDistance', outlier_disp_tolerance, ...
                                                'MaxNumTrials', ransac_iters);

% find indices of valid matches
[~, valid_match_indices] = ismember(inliers1_l, im_location1_l, 'rows');

% cleaned matched points after MSAC
cleaned_matches = matches(valid_match_indices);

end
