clc
clear
close all

%% Read calibration parameters P1 and P2 
%calibname = '../data/calib.txt';
%T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
%A = table2array(T);
 
%P1 = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
%P2 = vertcat(A(2,1:4), A(2,5:8), A(2,9:12));

A = [[9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00]; ...
     [9.895267e+02 0.000000e+00 7.020000e+02 0.000000e+00 9.878386e+02 2.455590e+02 0.000000e+00 0.000000e+00 1.000000e+00]];
K1=vertcat(A(1,1:3), A(1,4:6), A(1,7:9));
K2=vertcat(A(2,1:3), A(2,4:6), A(2,7:9));

%% Initialize variables
pos = [0;0;0];
Rpos = eye(3);

path1='../data/DataSet1/image_00/data/000000';
path2='../data/DataSet1/image_01/data/000000';

%% Read images
 
i=0;
dig1=imval2str(i);
dig2=imval2str(i+1);

I1_l = imread(strcat(path1,dig1,'.png'));
I1_r = imread(strcat(path2,dig1,'.png'));
I2_l = imread(strcat(path1,dig2,'.png'));
I2_r = imread(strcat(path2,dig2,'.png'));

%% Feature Tracking at time instant t
% In our implementation we have used the  Kanade-Lucas-Tomasi (KLT) algorithm
% to track the features in the left camera at time instant t

pts1_l=detectMinEigenFeatures(I1_l,'FilterSize',5,'MinQuality',0);
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, pts1_l.Location, I1_l);
[pts2_l, validity] = step(tracker, I2_l);

pts1_l(validity(:)==0,:) = [];
pts2_l(validity(:)==0,:) = [];
pts2_l=cornerPoints(pts2_l);

%% Feature points extraction
% In the original paper, filter masks followed by Non- Maximal Suppresion are
% used to detect the features. However, since the MinEigen algorithm works well
% for feature detection we have used its MATLAB implementation directly.

pts1_r=detectMinEigenFeatures(I1_r,'FilterSize',5,'MinQuality',0);
pts2_r=detectMinEigenFeatures(I2_r,'FilterSize',5,'MinQuality',0);

[features1_l,pts1_l] = extractFeatures(I1_l,pts1_l);
[features1_r,pts1_r] = extractFeatures(I1_r,pts1_r);
[features2_l,pts2_l] = extractFeatures(I2_l,pts2_l);
[features2_r,pts2_r] = extractFeatures(I2_r,pts2_r);

%figure;
%imshow(I2_l);
%hold on
%scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+b');

%% Circular matching
% In the original paper, sparse SSD circular matching has been done.
% However, to simplify the implementation of visual odometry we have
% done circular matching directly using the matchFeatures function in 
% MATLAB over Binary Features. 

% compare left frame at t with left frame at t-1
inPair = matchFeatures(features2_l,features1_l);

[features2_l,pts2_l] = extractFeatures(I2_l,pts2_l(inPair(:,1),:));
[features1_l,pts1_l] = extractFeatures(I1_l,pts1_l(inPair(:,2),:));

% compare left frame at t-1 with right frame at t-1
inPair = matchFeatures(features1_l,features1_r);

[~,pts1_l] = extractFeatures(I1_l,pts1_l(inPair(:,1),:));
[features1_r,pts1_r] = extractFeatures(I1_r,pts1_r(inPair(:,2),:));

% compare right frame at t-1 with right frame at t
inPair = matchFeatures(features1_r,features2_r);

[~,pts1_r] = extractFeatures(I1_r,pts1_r(inPair(:,1),:));
[features2_r,pts2_r] = extractFeatures(I2_r,pts2_r(inPair(:,2),:));

% compare right frame at t with left frame at t
inPair = matchFeatures(features2_r,features2_l);

[~,pts2_r] = extractFeatures(I2_r,pts2_r(inPair(:,1),:));
[~,pts2_l] = extractFeatures(I2_l,pts2_l(inPair(:,2),:));

%% Feature Selection using bucketing
bucketSize = 50;
numCorners = 3;

%hold on
%scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+r');
%pts2_l = bucketFeatures(I2_l,pts2_l,bucketSize,numCorners);
%scatter(pts2_l.Location(:,1),pts2_l.Location(:,2),'+g');

%% Feature Matching to get corresponding points at time instant t and t-1 in left camera
[features2_l,pts2_l] = extractFeatures(I2_l,pts2_l);
[features1_l,pts1_l] = extractFeatures(I1_l,pts1_l);
inPair = matchFeatures(features1_l,features2_l);

[~,pts1_l] = extractFeatures(I1_l,pts1_l(inPair(:,1),:));
[~,pts2_l] = extractFeatures(I2_l,pts2_l(inPair(:,2),:));

%figure; showMatchedFeatures(I2_l, I1_l, pts2_l, pts1_l);

%% Rotation Estimation using Nister's Algorithm

% Writing image coordinates in homogenous form
pts1_l = [pts1_l.Location'; ones(1,length(pts1_l))]; 
pts2_l = [pts2_l.Location'; ones(1,length(pts2_l))]; 
numPoints = length(pts1_l(1,:));

%Implement RANSAC algorithm to exclude outliers
%   s: smallest number of points required to fit the model
%   N: Number of iterations required to find model using RANSAC
%   w: Percentage number of inliers desired 
%   p: Accuracy in fitted line desired
s = 5;
w = 0.9;
p = 0.9999;
N = floor(log(1-p)/log(1-w^s));

minError=1000;
for i = 1:N
    sample = randperm(numPoints, 5);
    [E_all,R_all,~,~] = fivePointAlgorithm(pts1_l(:,sample), pts2_l(:,sample), K1, K1 );
    error = zeros(1,size(E_all,1));
    
    % Reprojection Error
    for j = 1:size(E_all,1)
        for k = 1:numPoints
            error(j) = error(j) + pts2_l(:,k)'*inv(K1)'*E_all{j}*inv(K1)*pts1_l(:,k);
        end
    end
    [~,index] = min(abs(error));
    if error(index) < minError
        R = R_all{index};
        minError=error(index);
    end
end
%% Plot the odometry transformed data
%pos = pos + Rpos*t;
%Rpos = R*Rpos;
%plot(pos(1),pos(2),'ob');
%hold on;
%pause(0.005);