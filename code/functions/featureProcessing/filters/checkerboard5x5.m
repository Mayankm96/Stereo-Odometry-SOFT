function out = checkerboard5x5(img)
%SOBEL5X5 Apply checkerboard filter over input image. Filter is given by:
%
%   -1 -1  0  1  1
%   -1 -1  0  1  1
%    0  0  0  0  0
%    1  1  0 -1 -1
%    1  1  0 -1 -1
%
% INPUT:
%   - img(image): Given input image
%
% OUTPUT:
%   - out: convolved output image 

checkerboard_filter = [-1 -1  0  1  1; ...
                       -1 -1  0  1  1; ...
                        0  0  0  0  0; ...
                        1  1  0 -1 -1; ...
                        1  1  0 -1 -1];
% convolve image with filter
out = conv2(img, checkerboard_filter, 'same');

end
