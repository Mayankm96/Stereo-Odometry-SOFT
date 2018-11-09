function [R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K1,K2)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C1_C0 = T_C1_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 0)
%   to camera 1.
%

% Count number of points lying in front of image plane for possible
% disambiguities
count_front = zeros(size(Rots, 3), 1);

for i = 1:size(Rots, 3)
    M1 = K1 * horzcat(eye(3), zeros(3, 1));
    M2 = K2 * horzcat(Rots(:, :, i), u3);
    pts_3D = linearTriangulation(p1, p2, M1, M2);
    
    for pt = pts_3D
        if pt(3) > 0
            count_front(i) = count_front(i) + 1;
        end
    end
end

% Find index corresponding to the case with maximum number of points in
% front of image plane
[~, index] = max(count_front);
% Take solution corresponding to the maximum count
R = Rots(:, :, index);
T = u3;
