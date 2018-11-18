function showStereoMatches(I1, I2, matches, current_time_flag, style, lineWidth,  ptSyle, ptWidth)
%SHOWSTEREOMATCHES Plot matches detected in two images using MATLAB
%
% INPUT: 
%   - I2_l: left image at time t
%   - I2_r: right image at time t
%   - matches(1, N): array of structure conatining keypoints 
%   - current_time_flag: flag for matches in previous time (=0) or current time (=1)
%   - style: defines color and type of plotted line
%   - lineWidth: defines thickness of plotted line
%   - ptSyle: defines type of points; eg: 'o'
%   - ptWidth: defines thickness of plotted points

% false color
imgOverlay = imfuse(I2, I1);
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
    if(current_time_flag)
        x_from(num) = match.pt2_r.location(1);
        x_to(num)   = match.pt2_l.location(1);
        y_from(num) = match.pt2_r.location(2);
        y_to(num)   = match.pt2_l.location(2);
    else
        x_from(num) = match.pt1_r.location(1);
        x_to(num)   = match.pt1_l.location(1);
        y_from(num) = match.pt1_r.location(2);
        y_to(num)   = match.pt1_l.location(2);
    end
end

% plot line
plot([y_from; y_to], [x_from; x_to], style, 'Linewidth', lineWidth);
% plot end points
scatter(y_from, x_from, ptSyle, 'r', 'Linewidth', ptWidth);
scatter(y_to, x_to, ptSyle, 'g', 'Linewidth', ptWidth);
end
