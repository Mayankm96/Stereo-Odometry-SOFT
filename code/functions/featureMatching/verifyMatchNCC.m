function validity = verifyMatchNCC(I1, I2, pt1, pt2, template_size, threshold)
%FINDMATCHNCC Given a keypoint in image 1, use normalized cross-correlation
% (NCC) to verify its match with a keypoint in image 2
%
% INPUT:
%   - I1(H, W): grayscale image 1
%   - I2(H, W): grayscale image 2
%   - pt1(1, 2): keypoint location in image 1
%   - pt2(1, 2): keypoint location in image 2
%   - template_size: window size of the patch for normalized cross-correlation
%   - threshold: threshold for normalized cross-correlation% OUTPUT:
% 
% OUTPUT:
%   - validity: true if the NCC score is above threshold

% templates
template1 = double(I1(ceil(pt1(1) - template_size/2) : ceil(pt1(1) + template_size/2), ...
           ceil(pt1(2) - template_size/2) : ceil(pt1(2) + template_size/2)));
template2 = double(I2(ceil(pt2(1) - template_size/2) : ceil(pt2(1) + template_size/2), ...
           ceil(pt2(2) - template_size/2) : ceil(pt2(2) + template_size/2)));

if std(template2(:)) == 0 || std(template1(:)) == 0
    validity = false;
    return;
end

score = normxcorr2(template1, template2);

if(max(score(:)) > threshold)
    validity = true;
else
    validity = false;
end

end

