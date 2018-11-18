function keypts_with_descriptors = computeFeatures(I, feature_params)
%COMPUTEFEATURES Compute sparse set of features from image
% 
% Input:
%   - I(image): input grayscale image 
%   - feature_params: structure containing the following:
%       - nms_n: non-max neighborhood
%       - nms_tau: non-max threshold
%       - margin: leaving margin for safety 
%
% Output: 
%   - keypts_with_descriptors(1, N): Extracted features keypoints with properties 
%                   [location(x, y), value, class, descriptor (128 bits)]

% feature processing parameters
nms_n = feature_params.nms_n;
nms_tau = feature_params.nms_tau;
margin = feature_params.margin;

% apply filters
[I_dx, I_dy] = sobel5x5(I);
I_f1 = blob5x5(I);
I_f2 = checkerboard5x5(I);

% extract keypoints via non-maximum suppression
keypts = nonMaximumSuppression(I_f1, I_f2, nms_n, nms_tau, margin);
keypts_with_descriptors = computeDescriptors(I_dx, I_dy, keypts);

end
