function cost = distPoint2EpipolarLine(F,p1,p2)
% distPoint2EpipolarLine  Compute the point-to-epipolar-line distance
%
%   Input:
%   - F(3,3): Fundamental matrix
%   - p1(3,NumPoints): homogeneous coords of the observed points in image 1
%   - p2(3,NumPoints): homogeneous coords of the observed points in image 2
%
%   Output:
%   - cost: sum of squared distance from points to epipolar lines
%           normalized by the number of point coordinates

NumPoints = size(p1,2);

homog_points = [p1, p2];
epi_lines = [F.'*p2, F*p1];

denom = epi_lines(1,:).^2 + epi_lines(2,:).^2;
cost = sqrt( sum( (sum(epi_lines.*homog_points,1).^2)./denom ) / NumPoints );
