function points3D = gen3dPoints(pts_l,pts_r,Pl,Pr)
%%Given matched points in the left and right frames of the stereo system, and 
% projection matrices for each camera, a 3D point cloud is generated.
%   pts_l: matched feature points locations in left camera frame
%   pts_r: matched feature points locations in right camera frame
%   Pl, Pr - 3x4 Projection matrices for left and right cameras respectively

% Writing image coordinates in homogenous form
pts_l = pts_l.Location'; 
pts_r = pts_r.Location'; 

% initializing homogeneous world coordinates
points3D = ones(size(pts_l,1),4); 

for i = 1:size(pts_l,1)
    pointInImage1 = pts_l(i,:);
    pointInImage2 = pts_r(i,:);
    
    A = [pointInImage1(1)*Pl(3,:) - Pl(1,:);
        pointInImage1(2)*Pl(3,:) - Pl(2,:);
        pointInImage2(1)*Pr(3,:) - Pr(1,:);
        pointInImage2(2)*Pr(3,:) - Pr(2,:)];

    % Compute the 3-D location using the smallest singular value from the
    % singular value decomposition of the matrix A
    [~,~,V]=svd(A);

    X = V(:,end);
    X = X/X(end);

    % Store location
    points3D(i,:) = X';

end
points3D = points3D(:,1:3);
end

