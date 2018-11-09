function [newpts, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points 
% so that their centroid is at the origin and their mean distance from 
% the origin is sqrt(2).
%
% Usage:   [newpts, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   newpts -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, newpts = T*pts
%

N = size(pts, 2);

% calculate statistic
pts_cart = pts(1:2, :);
mu = mean(pts_cart, 2);
std = 1/N * sum(sum((pts_cart - mu).^2, 1));
s = sqrt(2) / std;

% normalizing matrix
T = [s, 0, -s * mu(1); ...
      0, s, -s * mu(2); ...
      0, 0, 1];
  
% normalized points
newpts = zeros(size(pts));
for i =1:N
    newpts(:, i) = T * pts(:, i);
end  
