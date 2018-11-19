clc
clear
close all

%% Execute the configuration file to read parameters for data paths
addpath('config');
addpath(genpath('functions'));
configFile;

%% Starting parallel pooling (requires Parallel Processing Toolbox)
% This section takes a while to load for the first time
% To shutdown, run: delete(gcp('nocreate'));
if (isempty(gcp) && data_params.use_multithreads)
    parpool();
end

%% Read directories containing images
img_files1 = dir(strcat(data_params.path1,'*.png'));
img_files2 = dir(strcat(data_params.path2,'*.png'));
num_of_images = length(img_files1);

%% Read camera parameters
[P1, P2] = createCamProjectionMatrices(cam_params);

%% Read ground truth file if flag is true
if data_params.show_gt_flag
  ground_truth = load(data_params.gt_file);
  gt_x_max = max(ground_truth(:, end - 8));
  gt_x_min = min(ground_truth(:, end - 8));
  gt_z_max = max(ground_truth(:, end));
  gt_z_min = min(ground_truth(:, end));
end

%% Initialize variables for odometry
pos = [0;0;0];
Rpos = eye(3);

%% Start Algorithm
start = 0;
for t = 1 : num_of_images
    %% Read images for time instant t
    I2_l = imread([img_files1(t+1).folder, '/', img_files1(t).name]);
    I2_r = imread([img_files2(t+1).folder, '/', img_files2(t).name]);
    fprintf('Frame: %i\n', t);

    %% Bootstraping for initialization
    if (start == 0)
        vo_previous.pts1_l = computeFeatures(I2_l, vo_params.feature);
        vo_previous.pts1_r = computeFeatures(I2_r, vo_params.feature);
        start = 1;
        I1_l = I2_l;
        I1_r = I2_r;
        fprintf('\n---------------------------------\n');
        continue;
    end

    %% Implement SOFT for time instant t+1
    [R, tr, vo_previous] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous);

    %% Estimated pose relative to global frame at t = 0
    pos = pos + Rpos * tr';
    Rpos = R * Rpos;

    %% Prepare frames for next iteration
    I1_l = I2_l;
    I1_r = I2_r;

    %% Plot the odometry transformed data
    subplot(2, 2, [2, 4]);

    % Read ground truth pose if flag is true
    if data_params.show_gt_flag
      axis([gt_x_min gt_x_max gt_z_min gt_z_max])
      T = reshape(ground_truth(t, :), 4, 3)';
      pos_gt = T(:, 4);
      scatter(pos_gt(1), pos_gt(3), 'r', 'filled');
      hold on;
    end
    scatter( - pos(1), pos(3), 'b', 'filled');
    title(sprintf('Odometry plot at frame %d', t))
    xlabel('x-axis (in meters)');
    ylabel('z-axis (in meters)');

    if data_params.show_gt_flag
        legend('Ground Truth Pose', 'Estimated Pose')
    else
        legend('Estimated Pose')
    end

    %% Pause to visualize the plot
    pause(0.0001);
    fprintf('\n---------------------------------\n');
end
