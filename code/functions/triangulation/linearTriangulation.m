function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

N = size(p1, 2);
P = zeros(4, N);
for i = 1:N
    % find cross product matrices
    p1_x = cross2Matrix(p1(:, i));
    p2_x = cross2Matrix(p2(:, i));
    % formulate triangulation matrix for: Ap = 0
    A1 = p1_x * M1;
    A2 = p2_x * M2;
    A = vertcat(A1, A2);
    % solving above problem using SVD
    [~, ~, V] = svd(A'*A);
    P(:, i) = V(:, end);
end

% normalize homogenous coordinates
P = bsxfun (@rdivide, P, P(4,:));
