%% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
%% -------------------------------------------------------------------------------

% Path to the directories containing images
data_params.path1 = '../data/kitti/00/image_0/';
data_params.path2 = '../data/kitti/00/image_1/';

% Path to calibration text file
data_params.calib_file = '../data/kitti/00/calib.txt';

% Path to groundtruth poses. Set flag to 1 to plot groundtruth as well
data_params.gt_file = '../data/kitti/poses/00.txt';
data_params.show_gt_flag = 1;

%% Read the calibration file to find parameters of the cameras
% TO-DO: Read from the calib_file instead

% calibration parameters for sequence 2010_03_09_drive_0000 
cam_params.fx = 7.215377e+02;                     % focal length (u-coordinate) in pixels
cam_params.cx = 6.095593e+02;                     % principal point (u-coordinate) in pixels
cam_params.fy = 7.215377e+02;                     % focal length (v-coordinate) in pixels
cam_params.cy = 1.728540e+02;                     % principal point (v-coordinate) in pixels
cam_params.base = 3.875744e+02;                   % baseline in meters (absolute value)

%% Parameters for Feature Extraction
vo_params.feature.nms_n = 5;                      % non-max-suppression: min. distance between maxima (in pixels)
vo_params.feature.nms_tau = 50;                   % non-max-suppression: interest point peakiness threshold

%% Parameters for Feature Matching
vo_params.matcher.match_binsize = 50;             % matching bin width/height (affects efficiency only)
vo_params.matcher.match_radius = 200;             % matching radius (du/dv in pixels)
vo_params.matcher.match_disp_tolerance = 2;       % dv tolerance for stereo matches (in pixels)
vo_params.matcher.match_uniqueness = 0.9;         % ratio between best and second best match
vo_params.matcher.outlier_disp_tolerance = 5;     % outlier removal: disparity tolerance (in pixels)
vo_params.matcher.outlier_flow_tolerance = 5;     % outlier removal: flow tolerance (in pixels)
vo_params.matcher.refinement = 1;                 % refinement (0=none,1=pixel,2=subpixel)
        
%% Paramters for Feature Selection using bucketing
vo_params.bucketing.max_features = 2;             % maximal number of features per bucket 
vo_params.bucketing.bucket_width = 50;            % width of bucket
vo_params.bucketing.bucket_height = 50;           % height of bucket

%% Paramters for motion estimation
vo_params.estim.ransac_iters = 200;              % number of RANSAC iterations
vo_params.estim.inlier_threshold = 2.0;          % fundamental matrix inlier threshold
vo_params.estim.reweighing = 1;                  % lower border weights (more robust to calibration errors)
