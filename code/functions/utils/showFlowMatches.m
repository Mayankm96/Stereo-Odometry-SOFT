function showFlowMatches(I1_l, I2_l, matches, lineStyle, lineWidth, ptSyle, ptWidth)
%SHOWFLOWMATCHES Plot matches detected in between two consecutive time instants
%
% INPUT:
%   - I2_l: left image at time t
%   - I2_r: right image at time t
%   - matches(1, N): array of structure conatining keypoints
%   - lineStyle: defines color and type of plotted line
%   - lineWidth: defines thickness of plotted line
%   - ptSyle: defines type of points; eg: 'o'
%   - ptWidth: defines thickness of plotted points

% false color
imgOverlay = imfuse(I1_l, I2_l);
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
    x_from(num) = match.pt1_l.location(1);
    x_to(num)   = match.pt2_l.location(1);
    y_from(num) = match.pt1_l.location(2);
    y_to(num)   = match.pt2_l.location(2);
end
% plot line
plot([y_from; y_to], [x_from; x_to], lineStyle, 'Linewidth', lineWidth);
% plot end points
scatter(y_from, x_from, ptSyle,'r', 'Linewidth', ptWidth);
scatter(y_to, x_to, ptSyle, 'g', 'Linewidth', ptWidth);

end
