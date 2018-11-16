function [min_ind, validity] = findMatch(keypt1, keypts2, bin_pos2, x_bin_num, y_bin_num, ...
                             match_params, flow)
%FINDMATCH Given a keypoint in image 1, we use sparse SAD to find a matching
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
%       - match_uniqueness: ratio between best and second best match
%   - flow: flag to consider optical flow (for stereo setting, set flow:  0)
%
% OUTPUT:
%   - min_ind: index of the keypoint matched in Image 2
%   - validity: boolean flag for successful match or not

% initialize variables
min_cost_1 = 10000000;
min_cost_2 = 10000000;
min_ind = 0;

% matching parameters
match_binsize = match_params.match_binsize;
match_radius = match_params.match_radius;
match_disp_tolerance = match_params.match_disp_tolerance;
match_uniqueness = match_params.match_uniqueness;

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
    y_min = y1 - match_disp_tolerance;
    y_max = y1 + match_disp_tolerance;
end

% bins of interest
x_bin_min = min( max( floor(x_min/match_binsize), 0), x_bin_num - 1) + 1;
x_bin_max = min( max( floor(x_max/match_binsize), 0), x_bin_num - 1) + 1;
y_bin_min = min( max( floor(y_min/match_binsize), 0), y_bin_num - 1) + 1;
y_bin_max = min( max( floor(y_max/match_binsize), 0), y_bin_num - 1) + 1;

% for all bins of interest do
 for x_bin = x_bin_min:x_bin_max
     for y_bin = y_bin_min:y_bin_max
         % consider keypoints in image 2 present in the associated bin
         for in2 = bin_pos2{x_bin, y_bin, c}
             % extract coordinates
             x2 = keypts2(in2).location(1);
             y2 = keypts2(in2).location(2);
             % check for validity of candidate to be within matching radius 
             if (x2 >= x_min && x2 <= x_max && y2 >= y_min && y2 <= y_max)
                 descr2 = keypts2(in2).descriptor;
                 % compute SAD score
                 dists = pdist2(double(descr1'), double(descr2'), 'cityblock');
                 cost = sum(dists);
                 % candidate keypoint in image 2 is the one correspodning
                 % to the least SAD score
                 if (cost < min_cost_1)
                    min_ind  = in2;
                    min_cost_1 = cost;
                 else 
                     if (cost < min_cost_2)
                         min_cost_2 = cost;
                     end
                 end
             end
         end
     end
 end
 
 % check uniqueness criterion
 if ( min_cost_1 <= match_uniqueness * min_cost_2)
     validity = true;
 else
     validity = false;
 end
         
end
