function plotFeatures(pts,  ptSyle, ptWidth, classColor)
%PLOTFEATURES Plot features using MATLAB
%
% INPUT:
%   - pts(1, N): array of structure conatining keypoints
%   - ptSyle: defines color and type of scatter plot; eg: '+r'
%   - ptWidth: defines thickness of scatter points
%   - classColor: color keypoints according to their class
assert (size(pts, 2) > size(pts, 1), 'Input expected to array of shape (1, N)')

% get locations of points
locations = vertcat(pts(:).location);
class_pt = vertcat(pts(:).class);

if(classColor)
    color = ['r', 'b', 'g', 'y'];
    for i = 1:4
        scatter(locations(class_pt == i, 2), locations(class_pt == i, 1), ptSyle, color(i), 'LineWidth', ptWidth);
        hold on;
    end
    legend('Blob mimimum', 'Blob maximum', 'Corner minimum', 'Corner maximum')
else
    scatter(locations(:, 2), locations(:, 1), ptSyle, 'LineWidth', ptWidth);
end

end
