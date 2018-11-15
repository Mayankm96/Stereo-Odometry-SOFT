function failed_min = checkMinimumValidity(I, fmin, fmin_i, fmin_j, nms_n, margin)
%CHECKMINIMUMVALIDITY 
%
% Input:
%   - I(image)
%   - fmin: value of the detected feature point
%   - fmin_i: x-coordinate of the detected feature point
%   - fmin_j: y-coordinate of the detected feature point
%   - nms_n: min. distance between maxima (in pixels)
%   - margin: additional distance for safety
%
% Output:
%   - failed_min: boolean variable; 1 if detected feature is invalid

% size of image
[width, height] = size(I);

% consider window around detcted feature point
window_row_index = (fmin_i - nms_n):min(fmin_i + nms_n, width - 1 - margin);
window_col_index = (fmin_j - nms_n):min(fmin_j + nms_n, height - 1 - margin);
I_window = I(window_row_index, window_col_index);

% find minima in this window
[minval, kp] = min(I_window(:));
[min_i, min_j] = ind2sub(size(I_window), kp);

% compare detected feature point with minima
if (minval < fmin && ( min_i < fmin_i || min_i > (fmin_i + nms_n) || min_j < fmin_j || min_j > (fmin_j + nms_n)))
    failed_min = true;
else
    failed_min = false;
end
    