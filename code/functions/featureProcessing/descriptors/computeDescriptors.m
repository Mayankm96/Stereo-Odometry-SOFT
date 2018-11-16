function keypts_out = computeDescriptors(I_dx, I_dy, keypts_in)
%COMPUTEDESCRIPTORS Computes ELAS descriptors for a given set of input
% keypoints and adds them as an additional property to keypoints.
% The descriptor is based on creating a generative probabilistic model for
% stereo matching that can be used for efficient fast matching.
%
% Source: "Efficient Large-Scale Stereo Matching"; Andreas Geiger, Martin
% Roser and Raquel Urtasun; ACCV'10
%
% INPUT:
%   - I_dx(image): gradient along x-direction
%   - I_dy(image): gradient along y-direction
%   - keypts_in(1, N): keypoints detected (location(x, y), value, class)
%
% OUTPUT:
%   - keypts_out(1, N):  keypoints with descriptors proerty added

% to speed-up matching, a sparse set of 16 locations are chosen from the
% horizontal and vertical Sobel responses
%   descriptor mask:
%       0  0  0  0  1  0  1  0  0  0  0
%       0  0  0  0  0  0  0  0  0  0  0
%       1  0  0  0  0  0  0  0  0  0  1
%       0  0  0  0  0  0  0  0  0  0  0
%       0  0  1  0  1  0  1  0  1  0  0
%       0  0  0  0  0  X  0  0  0  0  0
%       0  0  1  0  1  0  1  0  1  0  0
%       0  0  0  0  0  0  0  0  0  0  0
%       1  0  0  0  0  0  0  0  0  0  1
%       0  0  0  0  0  0  0  0  0  0  0
%       0  0  0  0  1  0  1  0  0  0  0
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

% initialize output keypts array
keypts_out = keypts_in;

for i = 1:size(keypts_in, 2)
    % extract keypoint location
    pt = keypts_in(i).location;
    x = pt(1); y = pt(2);
    patch_x = I_dx(x-5:x+5, y-5:y+5);
    patch_y = I_dy(x-5:x+5, y-5:y+5);
    % write the descriptor for the keypoint according to the horizontal
    % and vertical Sobel responses
    keypts_out(i).descriptor  = uint8([patch_x(descriptor_mask); patch_y(descriptor_mask)]);
end

end
