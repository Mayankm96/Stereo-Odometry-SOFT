function bin_pos = createIndexVector(keypts, match_binsize, x_bin_num, y_bin_num)
%CREATEINDEXVECTOR Find the bin position to which each keypoint belongs to
%
% INPUT:
%   - keypts(1, N): input feature keypoints
%   - match_binsize: matching bin width/height
%   - x_bin_num: number of bins along x-direction
%   - y_bin_num: number of bins along y-direction
%
% OUTPUT:
%   - bin_pos(x_bin_num, y_bin_num, 4): cell containing indices of keypoint

% allocate memory
num = size(keypts, 2);
% number of classes
classes_num = 4; 

% create a array for bin positions/class
bin_pos = cell(x_bin_num, y_bin_num, classes_num);

% iterate over all keypoints
for i = 1:num
    % coordinate of keypoint
    x = keypts(i).location(1);
    y = keypts(i).location(2);
    c = keypts(i).class;
    % find bin position
    bin_x = min(ceil(x/match_binsize), x_bin_num);
    bin_y = min(ceil(y/match_binsize), y_bin_num);
    % add keypoint index to corresponding bin_pos
    bin_pos{bin_x, bin_y, c} = horzcat(bin_pos{bin_x, bin_y, c}, i);
end

end
