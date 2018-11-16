function showFeatures(I, pts, style, linewidth)
%PLOTFEATURES Plot features using MATLAB
%
% INPUT: 
%   - I(image): image corresponding to the points
%   - pts(1, N): array of structure conatining keypoints 
%   - style: defines color and type of scatter plot. Ex: '+r'
%   - linewidth: defines thickness of scatter points

assert (size(pts, 2) > size(pts, 1), 'Input expected to array of shape (1, N)')

% get locations of points
loc_x = zeros(1, size(pts, 2));
loc_y = zeros(1, size(pts, 2));

for point = pts
    loc_x = [loc_x, point.location(1)];
    loc_y = [loc_y, point.location(2)];
end

% plot image
imshow(I);
hold on;
scatter(loc_y, loc_x, style, 'LineWidth', linewidth);

end

