function [maxima1, maxima2] = computeFeatures(I, nms_n, nms_tau, multi_stage)
%COMPUTEFEATURES Compute sparse set of features from image
% 
% Input:
%   - I(image): input grayscale image 
%   - nms_n: non-max neighborhood
%   - nms_tau: non-max threshold
%   - multi_stage: flag to enable sparse or/and dense feature extraction
%
% Output: 
%   - maxima1(N1, 1): Sparse maxima features with properties 
%                           [location(u, v), value, class, descriptor (128 bits)]
%   - maxima2(N2, 1): Dense maxima features with properties 
%                           [location(u, v), value, class, descriptor (128 bits)]

% apply filters
[I_dx, I_dy] = sobel5x5(I);
I_f1 = blob5x5(I);
I_f2 = checkerboard5x5(I);

% extract sparse maxima (1st pass) via non-maximum suppression
nms_n_sparse = nms_n * 3;
if (nms_n_sparse > 10)
  nms_n_sparse = max(nms_n, 10);
end
maxima1 = nonMaximumSuppression(I_f1, I_f2, nms_n_sparse, nms_tau);
descriptors = computeDescriptors(I_dx, I_dy, maxima1);
maxima1.descriptors = descriptors;

% extract dense maxima (2nd pass) via non-maximum suppression
maxima2 = [];
if multi_stage
    maxima2 = nonMaximumSuppression(I_f1, I_f2, nms_n, nms_tau);
    descriptors = computeDescriptors(I_dx, I_dy, maxima2);
    maxima2.descriptors = descriptors;
end

end
