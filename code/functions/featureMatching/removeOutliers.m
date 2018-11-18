function cleaned_matches = removeOutliers(I1_l, I1_r, matches, match_params)
%REMOVEOUTLIERS Summary of this function goes here
%
% INPUT:
%
%   - I1_l: left image for time t-1
%   - I1_r: right images for time t-1
%   - matches (1,N): strcutre of matched points
%       - pt1_l(2): location of keypoint in left image at time t-1
%       - pt1_r(2): location of keypoint in right image at time t-1
%       - pt2_l(2): location of keypoint in left image at time t
%       - pt2_r(2): location of keypoint in right image at time t
%   - match_params: structure comprising of following
%       - match_ncc_window: window size of the patch for normalized cross-correlation
%       - match_ncc_tolerance: threshold for normalized cross-correlation
% OUTPUT:
%   - cleaned_matches(1, N2): processed structure of matched points
%       - pt1_l(2): location of keypoint in left image at time t-1
%       - pt1_r(2): location of keypoint in right image at time t-1
%       - pt2_l(2): location of keypoint in left image at time t
%       - pt2_r(2): location of keypoint in right image at time t

% initialize parameters
ncc_window = match_params.match_ncc_window;
ncc_tolerance = match_params.match_ncc_tolerance;

% store indices of matches that are valid
valid_match_indices = ones(length(matches), 1, 'logical');

% for all matches do
parfor i = 1:length(matches)
    match = matches(i);
    % verify match by computing NCC score
    validity = verifyMatchNCC(I1_l, I1_r, match.pt1_l, match.pt1_r, ...
                              ncc_window, ncc_tolerance);
    % if not valid, remove the match
    if not(validity)
        valid_match_indices(i) = 0;
    end
end

% return array of cleaned matched points
cleaned_matches = matches(valid_match_indices);

end

