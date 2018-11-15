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

% convolve image with a (1,2,0,-2,-1) row vector
sobel_para = [1, 2, 0, -2, -1];
I_dx = conv2(img, sobel_para, 'same');

% convolve image with a (1,4,6,4,1) row vector
sobel_orth = [1, 4, 6, 4, 1];
I_dy = conv2(img, sobel_orth, 'same');
% scale output by 1/128
I_dy = I_dy / 128;
% clamp to range [-128, 128]
I_dy(I_dy < -128) = -128;
I_dy(I_dy >  128) =  128;
% shift output to [0, 255]
I_dy = I_dy + 128;

end
