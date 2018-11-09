%% Configuration File for main
%   - Permits various adjustments to parameters of the Visual Odometry Algorithm

% Path to the directories containing images
path1 = 'data/kitti/00/image_0/';
path2 = 'data/kitti/00/image_1/';

% Path to calibration text file
calib_file = 'data/kitti/00/calib.txt';

% Path to groundtruth poses. Set flag to 1 to plot groundtruth as well
gt_file = 'data/kitti/poses/00.txt';
show_gt_flag = 1;

%% Read the calibration file to find parameters of the cameras
% TO-DO: Read from the calib_file instead
A=[[7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00]; ...
   [7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00]];
P1 = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
P2 = vertcat(A(2,1:4), A(2,5:8), A(2,9:12));

K1 = P1(1:3, 1:3);
K2 = P2(1:3, 1:3);
