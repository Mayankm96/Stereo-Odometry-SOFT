function points3D = gen3dPoints(pts_l,pts_r,Pl,Pr)
%%Given matched points in the left and right frames of the stereo system, and 
% projection matrices for each camera, a 3D point cloud is generated.
%   pts_l: matched feature points locations in left camera frame
%   pts_r: matched feature points locations in right camera frame
%   Pl, Pr - 3x4 Projection matrices for left and right cameras respectively

% Retrieving image coordinates from datatype cornerPoints
pts_l = pts_l.Location; 
pts_r = pts_r.Location; 

points3D = triangulate(pts_l, pts_r, Pl', Pr')';
end

