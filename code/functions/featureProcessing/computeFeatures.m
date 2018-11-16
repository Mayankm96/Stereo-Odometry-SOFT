function [keypts] = computeFeatures(I, nms_n, nms_tau)
%COMPUTEFEATURES Compute sparse set of features from image
% 
% Input:
%   - I(image): input grayscale image 
%   - nms_n: non-max neighborhood
%   - nms_tau: non-max threshold
%
% Output: 
%   - keypts(1, N): Keypoints detected with properties 
%                       (location, value, class, descriptor (128 bits))

% apply filters
[I_dx, I_dy] = sobel5x5(I);
I_f1 = blob5x5(I);
I_f2 = checkerboard5x5(I);

% extract keypoint locations via non-maximum suppression
keypts = nonMaximumSuppression(I_f1, I_f2, nms_n, nms_tau);
% extract descriptors using Sobel responses
keypts = computeDescriptors(I_dx, I_dy, keypts);

end
