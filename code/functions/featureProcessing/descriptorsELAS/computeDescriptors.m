function descriptors = computeDescriptors(I_dx, I_dy, keypoints)
%COMPUTEDESCRIPTORS Computes ELAS descriptors for a given set of input
% keypoints. It is based on creating a generative probabilistic model for 
% stereo matching that can be used for efficient fast matching.
%
% Source: "Efficient Large-Scale Stereo Matching"; Andreas Geiger, Martin 
% Roser and Raquel Urtasun; ACCV'10
%
% INPUT:
%   - I_dx(image): gradient along x-direction
%   - I_dy(image): gradient along y-direction
%   - keypoints(N): keypoints detected ([location(x, y), value, class], count)
%
% OUTPUT:
%   - descriptors(N, 16): descriptors for input keypoints

N = keypoints.count;

% to speed-up matching, Sobel responses are quantized to 8 bits 
% and a sparse set of 16 locations are chosen from the horizontal and
% vertical Sobel responses
descriptors = zeros(N, 32, 'uint8');

% descriptor mask
%   0  0  0  0  1  0  1  0  0  0  0 
%   0  0  0  0  0  0  0  0  0  0  0 
%   1  0  0  0  0  0  0  0  0  0  1 
%   0  0  0  0  0  0  0  0  0  0  0 
%   0  1  0  1  0  1  0  1  0  1  0 
%   0  0  0  0  0  0  0  0  0  0  0 
%   0  1  0  1  0  1  0  1  0  1  0 
%   0  0  0  0  0  0  0  0  0  0  0 
%   1  0  0  0  0  0  0  0  0  0  1 
%   0  0  0  0  0  0  0  0  0  0  0 
%   0  0  0  0  1  0  1  0  0  0  0 
%   0  0  0  0  0  0  0  0  0  0  0 
descriptor_mask = zeros(11, 11, 'logical');
descriptor_mask(1, 5)   = 1;
descriptor_mask(1, 7)   = 1;
descriptor_mask(3, 1)   = 1;
descriptor_mask(3, 11)  = 1;
descriptor_mask(5, 3)   = 1;
descriptor_mask(5, 5)   = 1;
descriptor_mask(5, 7)   = 1;
descriptor_mask(5, 9)   = 1;
descriptor_mask(7, 3)   = 1;
descriptor_mask(7, 5)   = 1;
descriptor_mask(7, 7)   = 1;
descriptor_mask(7, 9)   = 1;
descriptor_mask(9, 1)   = 1;
descriptor_mask(9, 11)  = 1;
descriptor_mask(11, 5)  = 1;
descriptor_mask(11, 7)  = 1;

for i = 1:N
    % extract keypoint location
    pt = keypoints.location(i, :);
    u = pt(1); v = pt(2);
    patch_x = I_dx(u-5:u+5, v-5:v+5);
    patch_y = I_dy(u-5:u+5, v-5:v+5);
    % write the descriptor for the keypoint
    % for horizontal Sobel response
    descriptors(i, 1:16)  = patch_x(descriptor_mask);
    % for vertical Sobel response
    descriptors(i, 17:32) = patch_y(descriptor_mask);
end



