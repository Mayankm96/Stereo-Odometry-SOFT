function out = blob5x5(img)
%SOBEL5X5 Apply blob filter over input image. Filter is given by:
%
%   -1 -1 -1 -1 -1
%   -1  1  1  1 -1
%   -1  1  8  1 -1
%   -1  1  1  1 -1
%   -1 -1 -1 -1 -1
%
% INPUT:
%   - img(image): Given input image
%
% OUTPUT:
%   - out: convolved output image 

blob_filter = [-1 -1 -1 -1 -1; ...
               -1  1  1  1 -1; ...
               -1  1  8  1 -1; ...
               -1  1  1  1 -1; ...
               -1 -1 -1 -1 -1];
% convolve image with filter
out = conv2(img, blob_filter, 'same');

end
