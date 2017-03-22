clc
clear
% read calibration parameters (intrinsic and extrinsic) from the
% calibration file
calibname = '../sample_data/calib.txt';
T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
A = table2array(T);

P1 = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
P2 = vertcat(A(2,1:4), A(2,5:8), A(2,9:12));

I1_l = rgb2gray(imread('../sample_data/image_2/000000.png'));
I1_r = rgb2gray(imread('../sample_data/image_3/000000.png'));
I2_l = rgb2gray(imread('../sample_data/image_2/000001.png'));
I2_r = rgb2gray(imread('../sample_data/image_3/000001.png'));

