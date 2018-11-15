function failed_max = checkMaximumValidity(I, fmax, fmax_i, fmax_j, nms_n, margin)
%CHECKMAXIMUMVALIDITY 
%
% Input:
%   - I(image)
%   - fmax: value of the detected feature point
%   - fmax_i: x-coordinate of the detected feature point
%   - fmax_j: y-coordinate of the detected feature point
%   - nms_n: min. distance between maxima (in pixels)
%   - margin: additional distance for safety
%
% Output:
%   - failed_max: boolean variable; 1 if detected feature is invalid

% size of image
[width, height] = size(I);

% consider window around detcted feature point
window_row_index = (fmax_i - nms_n):min(fmax_i + nms_n, width - 1 - margin);
window_col_index = (fmax_j - nms_n):min(fmax_j + nms_n, height - 1 - margin);
I_window = I(window_row_index, window_col_index);

% find minima in this window
[maxval, kp] = max(I_window(:));
[max_i, max_j] = ind2sub(size(I_window), kp);

% compare detected feature point with minima
if (maxval > fmax && ( max_i < fmax_i || max_i > (fmax_i + nms_n) || max_j < fmax_j || max_j > (fmax_j + nms_n)))
    failed_max = true;
else
    failed_max = false;
end
    