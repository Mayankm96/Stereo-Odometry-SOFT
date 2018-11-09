function [R, u] = estimRotationAndTranslation(pts1, pts2, K1, K2, s, w, p )
%%Given matched points in the left and right frames of the stereo system, and
% parameters for RANSAC, rotation matrix is estimated using 8-Points algorithm
%   pts1: matched feature points locations in first camera frame
%   pts2: matched feature points locations in second camera frame
%   K1, K2 - 3x3 Intrinsic parameters matrices for the two cameras
%   s: smallest number of points required to fit the model
%   w: Percentage number of inliers desired
%   p: Accuracy in fitted line desired

% Writing image coordinates in homogenous form
pts1 = [pts1.Location'; ones(1,length(pts1))];
pts2 = [pts2.Location'; ones(1,length(pts2))];

% Number of iterations required to find rotation matrix using RANSAC
N = floor(log(1-p)/log(1-w^s));

% Number of matched feature points
numPoints = length(pts1(1,:));

minError=1000;
R_min = zeros(3, 3);

for i = 1:N
    error = 0;
    sample = randperm(numPoints, 5);
    
    % Estimate the essential matrix E using the 8-point algorithm
    E = estimateEssentialMatrix(pts1(:,sample), pts2(:,sample), K1, K2 );
    
    % Reprojection Error
    for k = 1:numPoints
        error = error + pts2(:,k)'* inv(K2)'* E * inv(K1) * pts1(:,k);
    end

    if error < minError
        % Obtain extrinsic parameters (R,t) from E
        [Rots,u3] = decomposeEssentialMatrix(E);

        % Disambiguate among the four possible configurations
        [R, ~] = disambiguateRelativePose(Rots, u3, pts1, pts2, K1, K2);
        R_min = R;
        u = u3;
        minError = error;
    end
end

end
