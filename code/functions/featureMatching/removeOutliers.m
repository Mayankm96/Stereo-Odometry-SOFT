function cleaned_matches = removeOutliers(I1_l, I1_r, I2_l, I2_r, matches, match_params)
%REMOVEOUTLIERS Performs outliers removal by applying normalized cross-correlation
% score between patches around the matched points.
%
% INPUT:
%
%   - I1_l: left image for time t-1
%   - I1_r: right images for time t-1
%   - I2_l: left image for time t
%   - I2_r: right images for time t
%   - matches (1,N): strcutre of matched points
%       - pt1_l: keypoint in left image at time t-1
%       - pt1_r: keypoint in right image at time t-1
%       - pt2_l: keypoint in left image at time t
%       - pt2_r: keypoint in right image at time t
%   - match_params: structure comprising of following
%       - match_ncc_window: window size of the patch for normalized cross-correlation
%       - match_ncc_tolerance: threshold for normalized cross-correlation
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

% store indices of matches that are valid
valid_match_indices = ones(length(matches), 1, 'logical');

% for all matches do
parfor i = 1:length(matches)
    match = matches(i);
    
    % verify match by computing NCC score
    % a) left frame at t-1 with right frame at t-1
    validity = verifyMatchNCC(I1_l, I1_r, match.pt1_l.location, match.pt1_r.location, ...
                                ncc_window, ncc_tolerance);
    if not(validity)
        valid_match_indices(i) = 0;
        continue;
    end
    
    % b) right frame at t-1 with right frame at t
    validity = verifyMatchNCC(I1_r, I2_r, match.pt1_r.location, match.pt2_r.location, ...
                                ncc_window, ncc_tolerance);
    if not(validity)
        valid_match_indices(i) = 0;
        continue;
    end
    
    % c) right frame at t with left frame at t
    validity = verifyMatchNCC(I2_r, I2_l, match.pt1_r.location, match.pt2_l.location, ...
                                ncc_window, ncc_tolerance);
    if not(validity)
        valid_match_indices(i) = 0;
        continue;
    end
    
    % d) left frame at t with left frame at t-1
    validity = verifyMatchNCC(I2_l, I1_l, match.pt2_l.location, match.pt1_l.location, ...
                                ncc_window, ncc_tolerance);
    if not(validity)
        valid_match_indices(i) = 0;
        continue;
    end                        
end

% cleaned matched points after NCC
cleaned_matches = matches(valid_match_indices);

end
