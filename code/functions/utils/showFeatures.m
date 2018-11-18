function showFeatures(I, pts,  ptSyle, ptWidth)
%PLOTFEATURES Plot features using MATLAB
%
% INPUT:
%   - I(image): image corresponding to the points
%   - pts(1, N): array of structure conatining keypoints
%   - ptSyle: defines color and type of scatter plot; eg: '+r'
%   - ptWidth: defines thickness of scatter points

assert (size(pts, 2) > size(pts, 1), 'Input expected to array of shape (1, N)')

% get locations of points
loc_x = zeros(1, size(pts, 2));
loc_y = zeros(1, size(pts, 2));
class_pt = zeros(1, size(pts, 2));

index = 1;
for point = pts
    index = index + 1;
    loc_x(index) = point.location(1);
    loc_y(index) = point.location(2);
    class_pt(index) = point.class;
end

color = ['r', 'b', 'g', 'y'];

% plot image
imshow(I);
hold on;
for i = 1:4
    scatter(loc_y(class_pt == i), loc_x(class_pt == i), ptSyle, color(i), 'LineWidth', ptWidth);
end
legend('Blob mimimum', 'Blob maximum', 'Corner minimum', 'Corner maximum')

end
