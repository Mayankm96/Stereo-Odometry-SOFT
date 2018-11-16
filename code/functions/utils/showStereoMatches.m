function showStereoMatches(I2_l, I2_r, matches, style, lineWidth, ptWidth)
%SHOWSTEREOMATCHES Plot matches detected in two images using MATLAB
%
% INPUT: 
%   - I2_l: left image at time t
%   - I2_r: right image at time t
%   - matches(1, N): array of structure conatining keypoints 
%   - style: defines color and type of plotted line
%   - lineWidth: defines thickness of plotted line
%   - ptWidth: defines thickness of plotted points

% false color
imgOverlay = imfuse(I2_r, I2_l);
% create red-cyan image instead of the imfuse default
imgOverlay(:,:,1) = imgOverlay(:,:,2);
imgOverlay(:,:,2) = imgOverlay(:,:,3);

% show image
imshow(imgOverlay);
hold on

% for all matches
x_from = zeros(1, size(matches, 2));
x_to = zeros(1, size(matches, 2));
y_from = zeros(1, size(matches, 2));
y_to = zeros(1, size(matches, 2));
num = 0;
for match = matches
    num = num + 1;
    x_from(num) = match.pt2_r(1);
    x_to(num)   = match.pt2_l(1);
    y_from(num) = match.pt2_r(2);
    y_to(num)   = match.pt2_l(2);
end
% plot end points
scatter(y_from, x_from, '+r', 'Linewidth', ptWidth);
scatter(y_to, x_to, '+g', 'Linewidth', ptWidth);
% plot line
plot([y_from; y_to], [x_from; x_to], style, 'Linewidth', lineWidth);

end
