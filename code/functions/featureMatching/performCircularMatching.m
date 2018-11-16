function matches = performCircularMatching(pts1_l, pts2_l, pts1_r, pts2_r, dims, match_params)
%%performCircularMatching Given feature points in a pair of stereo images at
% time instants t-1 and t, perform circular matching using SAD algorithm
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
%
% OUTPUT:
%   - matches: array of structure of matched points across four images
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t

% size of image
height = dims(1);
width = dims(2);

% matching parameters
match_binsize = match_params.match_binsize;

% calculate number of bins
x_bin_num = floor(height/match_binsize);
y_bin_num = floor(width/match_binsize);

% create position/class bin index vectors
bin_pos1_l = createIndexVector(pts1_l, match_binsize, x_bin_num, y_bin_num);
bin_pos1_r = createIndexVector(pts1_r, match_binsize, x_bin_num, y_bin_num);
bin_pos2_l = createIndexVector(pts2_l, match_binsize, x_bin_num, y_bin_num);
bin_pos2_r = createIndexVector(pts2_r, match_binsize, x_bin_num, y_bin_num);

% initialize variables
matches = struct('pt1_l', {}, 'pt1_r', {}, 'pt2_l', {}, 'pt2_r', {});
num = 0;

% iterate over all keypoints observations
for i1_l = 1 : size(pts1_l, 2)
    % match in circle
    % a) left frame at t-1 with right frame at t-1
    [i1_r, validity] = findMatch(pts1_l(i1_l), pts1_r, bin_pos1_r, x_bin_num, y_bin_num, match_params, 0);
    if not(validity)
        continue;
    end
    % b) right frame at t-1 with right frame at t
    [i2_r, validity] = findMatch(pts1_r(i1_r), pts2_r, bin_pos2_r, x_bin_num, y_bin_num, match_params, 1);
    if not(validity)
        continue;
    end
    % c) right frame at t with left frame at t
   [i2_l, validity] = findMatch(pts2_r(i2_r), pts2_l, bin_pos2_l, x_bin_num, y_bin_num, match_params, 0);
    if not(validity)
        continue;
    end
    % d) left frame at t with left frame at t-1
    [i1_l2, validity] = findMatch(pts2_l(i2_l), pts1_l, bin_pos1_l, x_bin_num, y_bin_num, match_params, 1);
    if not(validity)
        continue;
    end
    
    % if succcessfull circular matching
    if(i1_l == i1_l2)
        % retrieve all image coordinates of matched feature points
        m_pt1_l =  pts1_l(i1_l).location;
        m_pt2_l =  pts2_l(i2_l).location;
        m_pt1_r =  pts1_r(i1_r).location;
        m_pt2_r =  pts2_r(i2_r).location;
        
        % calculate disparity
        disp1 = m_pt1_l(2) - m_pt1_r(2);
        disp2 = m_pt2_l(2) - m_pt2_r(2);
      
        % if disparities are positive
        if (disp1 > 0 && disp2 > 0)
            % pull matches to the same scanline
            % for time t-1
            m_pt1_l(1) = (m_pt1_l(1) + m_pt1_r(1)) / 2;
            m_pt1_r(1) = m_pt1_l(1);
            % for time t
            m_pt2_l(1) = (m_pt2_l(1) + m_pt2_r(1)) / 2;
            m_pt2_r(1) = m_pt2_l(1);
            
            % add match
            num = num + 1;
            matches(num).pt1_l = m_pt1_l;
            matches(num).pt2_l = m_pt2_l;
            matches(num).pt1_r = m_pt1_r;
            matches(num).pt2_r = m_pt2_r;
        end
    end
end
    
end
