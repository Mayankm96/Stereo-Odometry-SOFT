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
descriptors = uint8([N, 32]);

for i = 1:N
    % extract keypoint location
    pt = keypoints.location(i, :);
    u = pt(1); v = pt(2);
    % write the descriptor for the keypoint
    % for horizontal Sobel response
    descriptors(i, 1) = I_dx(u-5, v-1);
    descriptors(i, 2) = I_dx(u-5, v+1);
    descriptors(i, 3) = I_dx(u-3, v-5);
    descriptors(i, 4) = I_dx(u-3, v+5);
    descriptors(i, 5) = I_dx(u-1, v-3);
    descriptors(i, 6) = I_dx(u-1, v-1);
    descriptors(i, 7) = I_dx(u-1, v+1);
    descriptors(i, 8) = I_dx(u-1, v+3);
    descriptors(i, 9) = I_dx(u+1, v-3);
    descriptors(i, 10) = I_dx(u+1, v-1);
    descriptors(i, 11) = I_dx(u+1, v+1);
    descriptors(i, 12) = I_dx(u+1, v+3);
    descriptors(i, 13) = I_dx(u+3, v-5);
    descriptors(i, 14) = I_dx(u+3, v+5);
    descriptors(i, 15) = I_dx(u+5, v-1);
    descriptors(i, 16) = I_dx(u+5, v+1);
    % for vertical Sobel response
    descriptors(i, 17) = I_dy(u-5, v-1);
    descriptors(i, 18) = I_dy(u-5, v+1);
    descriptors(i, 19) = I_dy(u-3, v-5);
    descriptors(i, 20) = I_dy(u-3, v+5);
    descriptors(i, 21) = I_dy(u-1, v-3);
    descriptors(i, 22) = I_dy(u-1, v-1);
    descriptors(i, 23) = I_dy(u-1, v+1);
    descriptors(i, 24) = I_dy(u-1, v+3);
    descriptors(i, 25) = I_dy(u+1, v-3);
    descriptors(i, 26) = I_dy(u+1, v-1);
    descriptors(i, 27) = I_dy(u+1, v+1);
    descriptors(i, 28) = I_dy(u+1, v+3);
    descriptors(i, 29) = I_dy(u+3, v-5);
    descriptors(i, 30) = I_dy(u+3, v+5);
    descriptors(i, 31) = I_dy(u+5, v-1);
    descriptors(i, 32) = I_dy(u+5, v+1);
end


