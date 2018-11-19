function P = linearTriangulation(pts1, pts2, P1, P2)
% LINEARTRIANGULATION  Linear Triangulation of matched points in two images
%
% Input:
%  - pts1(3,N): homogeneous coordinates of points in image 1
%  - pts2(3,N): homogeneous coordinates of points in image 2
%  - P1(3,4): projection matrix corresponding to first image
%  - P2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(3,N): homogeneous coordinates of 3-D points

N = size(pts1, 2);
P = zeros(4, N);

% for all points
for i = 1:N
    % find cross product matrices
    pt1_x = cross2Matrix(pts1(:, i));
    pt2_x = cross2Matrix(pts2(:, i));
    % formulate triangulation matrix for: Ap = 0
    A1 = pt1_x * P1;
    A2 = pt2_x * P2;
    A = vertcat(A1, A2);
    % solving above problem using SVD
    [~, ~, V] = svd(A'*A);
    P(:, i) = V(:, end);
end

% normalize homogenous coordinates
P = bsxfun (@rdivide, P, P(4,:));
P = P(1:3, :);

end
