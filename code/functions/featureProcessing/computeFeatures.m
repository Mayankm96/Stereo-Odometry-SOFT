function [keypts_sparse, keypts_dense] = computeFeatures(I, nms_n, nms_tau, multi_stage)
%COMPUTEFEATURES Compute sparse set of features from image
% 
% Input:
%   - I(image): input grayscale image 
%   - nms_n: non-max neighborhood
%   - nms_tau: non-max threshold
%   - multi_stage: flag to enable sparse or/and dense feature extraction
%
% Output: 
%   - keypts_sparse(1, N1): Sparse maxima features with properties 
%                       (location, value, class, descriptor (128 bits))
%   - keypts_dense(1, N2): Dense maxima features with properties 
%                       (location, value, class, descriptor (128 bits))

% apply filters
[I_dx, I_dy] = sobel5x5(I);
I_f1 = blob5x5(I);
I_f2 = checkerboard5x5(I);

% extract sparse maxima (1st pass) via non-maximum suppression
nms_n_sparse = nms_n * 3;
if (nms_n_sparse > 10)
  nms_n_sparse = max(nms_n, 10);
end
keypts_sparse = nonMaximumSuppression(I_f1, I_f2, nms_n_sparse, nms_tau);
keypts_sparse = computeDescriptors(I_dx, I_dy, keypts_sparse);

% extract dense maxima (2nd pass) via non-maximum suppression
keypts_dense = [];
if multi_stage
    keypts_dense = nonMaximumSuppression(I_f1, I_f2, nms_n, nms_tau);
    keypts_dense = computeDescriptors(I_dx, I_dy, keypts_dense);
end

end
