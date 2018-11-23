function [min_ind, validity] = findMatchSAD(keypt1, keypts2, bin_pos2, x_bin_num, y_bin_num, ...
                             match_params, flow)
%FINDMATCHSAD Given a keypoint in image 1, use sparse SAD to find a matching
% feature in image 2 within a (M × M) search window
%
% INPUT:
%   - keypt1: keypoint in Image 1 being considered
%   - keypts2(1, M): input feature keypoints detected in Image 2
%   - bin_pos2: positions/class of all keypoints in Image 2
%   - x_bin_num: number of bins in x-direction
%   - y_bin_num: number of bins in y-direction
%   - match_params: structure comprising of following
%       - match_binsize: matching bin width/height (for computation efficiency)
%       - match_radius: matching radius (du/dv in pixels)
%       - match_disp_tolerance: dv tolerance for stereo matches (in pixels)
%   - flow: flag to consider optical flow (for stereo setting, set flow:  0)
%
% OUTPUT:
%   - min_ind: index of the keypoint matched in Image 2
%   - validity: boolean flag for successful match or not

% matching parameters
match_binsize = match_params.match_binsize;
match_radius = match_params.match_radius;
match_disp_tolerance = match_params.match_disp_tolerance;

% extract information of feature being considered
pt1 = keypt1.location;
x1 = pt1(1); y1 = pt1(2);
c = keypt1.class;
descr1 = keypt1.descriptor;

% use full search space
x_min = x1 - match_radius;
x_max = x1 + match_radius;
y_min = y1 - match_radius;
y_max = y1 + match_radius;

% if stereo search, constrain to 1d since inputs are rectified
if not(flow)
    x_min = x1 - match_disp_tolerance;
    x_max = x1 + match_disp_tolerance;
end

% bins of interest
x_bin_min = min( max( ceil(x_min/match_binsize), 1), x_bin_num);
x_bin_max = min( max( ceil(x_max/match_binsize), 1), x_bin_num);
y_bin_min = min( max( ceil(y_min/match_binsize), 1), y_bin_num);
y_bin_max = min( max( ceil(y_max/match_binsize), 1), y_bin_num);

% consider keypoints in image 2 present in the bins of interest
in2 = horzcat(bin_pos2{x_bin_min:x_bin_max, y_bin_min:y_bin_max, c});

% if no valid indices => match does not exist
if (isempty(in2))
    min_ind = -1;
    validity = false;
    return;
end

% check if candidate are within matching radius
location2_all = vertcat(keypts2(in2).location)';
valid_in2_indices = find(location2_all(1, :) >= x_min & location2_all(1, :) <= x_max & ...
                 location2_all(2, :) >= y_min & location2_all(2, :) <= y_max);

% if no valid indices => match does not exist
if (isempty(valid_in2_indices))
    min_ind = -1;
    validity = false;
    return;
end

% extract descriptors
valid_in2 = in2(valid_in2_indices);
descr2_all = horzcat(keypts2(valid_in2).descriptor);

% compute SAD score between input keypoint in image 1 and all valid
% keypoints in image 2
dists = pdist2(double(descr1'), double(descr2_all'), 'cityblock');

% find keypoint in image2 associated with minimum cost
[~, indices] = sort(dists, 'ascend');
min_ind = valid_in2(indices(1));
validity = true;

end
