function [I_dx, I_dy] = sobel3x3(img)
%SOBEL3X3 Apply Sobel–Feldman filter over input image
% Sobel filters are discrete differentiation operator. They are used to 
% compute an approximation of the gradient of an image intensity function
%
% INPUT:
%   - img(image): Given input image
%
% OUTPUT:
%   - I_dx: gradient of image along x-axis
%   - I_dy: gradient of image along y-axis

[I_dx,I_dy] = imgradientxy(img, 'sobel');

% scaling gradients to be within the range [0, 255]
% scale output by 1/48
I_dy = I_dy / 48;
I_dx = I_dx / 48;
% shift output to [0, 255]
I_dx = uint8(I_dx + 128);
I_dy = uint8(I_dy + 128);

% imshowpair(I_dx, I_dy, 'montage');
end