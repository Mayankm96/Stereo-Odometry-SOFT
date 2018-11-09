function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix

N = size(p1, 2);

% create Q matrix for problem Q vec(F) = 0
Q = [];
for i = 1:N
    % calculating Kronecker product
    Q_i = kron(p1(:, i), p2(:, i));
    % concatenating to the system of equation
    Q = vertcat(Q, Q_i');
end
% obtaining solution using SVD
[~, ~, V] = svd(Q);
F = V(:, end);
F = reshape(F, 3, 3);
% forcing that det(F) = 0
[U, S, V] = svd(F);
S(3, 3) = 0;
F = U*S*V';
