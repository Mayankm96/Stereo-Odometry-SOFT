function [R,u3] = decomposeEssentialMatrix(E)
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R
% 
% Input:
%   - E(3,3) : Essential matrix
%
% Output:
%   - R(3,3,2) : the two possible rotations
%   - u3(3,1)   : a vector with the translation information

[U, ~, V] = svd(E);

u3 = V(:, end);

W = [0, -1, 0;...
     1, 0, 0;...
     0, 0, 1];
 
 R = zeros(3, 3, 2);
 R(:, :, 1) = U * W * V';
 R(:, :, 2) = U * W' * V';
 
 for i = 1:2
     if det(R(:,:, i)) < 0
         R(:,:, i) = - R(:,:, i);
     end
 end
 