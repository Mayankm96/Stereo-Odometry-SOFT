function well_matches = bucketFeatures(matches, bucketing_params)
% BUCKETFEATURE In every block (roughly ~ bucketSize x bucketSize) of the 
% image I, at most numCorner number of strongest features are chosen. This
% ensures a uniform distribution of  features over the image. 
%
% INPUT:
%   - matches(N1, 1):
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t
%   - bucketing_params: 
%       - bucket_width: width of bucket
%       - bucket_height: height of bucket
%       - max_features: maximal number of features per bucket 
%
% OUTPUT:
%   - well_matches(N2, 1):
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t

% Intialize parameters
bucket_width = bucketing_params.bucket_width;
bucket_height = bucketing_params.bucket_height;
max_features = bucketing_params.max_features;

% feature points in left image of time t-1
m_pts2_l = horzcat(matches(:).pt2_l);
% get locations of all matched points
locations2_l = vertcat(m_pts2_l(:).location)';

% find max values
x_max = max(locations2_l(1, :));
y_max = max(locations2_l(2, :));

% allocate number of buckets needed
bucket_cols = ceil(y_max/bucket_width);
bucket_rows = ceil(x_max/bucket_height);

% create a array for bucket positions/class
buckets = cell(bucket_rows, bucket_cols);

% iterate over all keypoints
for i = 1:size(m_pts2_l, 2)
    % coordinate of keypoint
    x = m_pts2_l(i).location(1);
    y = m_pts2_l(i).location(2);
    % find bin position
    bin_x = ceil(x/bucket_height);
    bin_y = ceil(y/bucket_width);
    % add keypoint index to corresponding bin_pos
    buckets{bin_x, bin_y} = horzcat(buckets{bin_x, bin_y}, i);
end

% refill matches from buckets
indices = [];
for index = 1:length(buckets(:))
    [pos_x, pos_y] = ind2sub(size(buckets), index);
    if numel(buckets{pos_x, pos_y}) >= 2
        indices = horzcat(indices, datasample(buckets{pos_x, pos_y}, max_features,'Replace', false));
    else
        if numel(buckets{pos_x, pos_y}) == 1
            indices = horzcat(indices, buckets{pos_x, pos_y});
        end
    end
end

well_matches = matches(indices);
end