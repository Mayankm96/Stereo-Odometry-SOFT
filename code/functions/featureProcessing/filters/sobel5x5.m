function [I_dx, I_dy] = sobel5x5(img)
%SOBEL5X5 Apply Sobel–Feldman filter over input image
% Sobel filters are discrete differentiation operator. They are used to 
% compute an approximation of the gradient of an image intensity function
%
% INPUT:
%   - img(image): Given input image
%
% OUTPUT:
%   - I_dx: gradient of image along x-axis
%   - I_dy: gradient of image along y-axis

% sobel kernels
sobel_kernel1 = [1, 4, 6, 4, 1];
sobel_kernel2 = [1, 2, 0, -2, -1];
divisor = 48;

% gradient along x
I_dx = conv2(sobel_kernel2', sobel_kernel1, img, 'valid');
I_dx = uint8(I_dx / divisor + 128);

% gradient along y
I_dy = conv2(sobel_kernel1', sobel_kernel2, img, 'valid');
I_dy = uint8(I_dy / divisor + 128);

% imshowpair(I_dx, I_dy, 'montage');
end
