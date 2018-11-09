clc
clear
close all

%% Execute the configuration file to read parameters for data paths
addpath('config');
configFile1;

%% Read ground truth file if flag is true
if show_gt_flag
  ground_truth = load(gt_file);
  ground_truth = ground_truth(:, [end-8 end]);
end

%% Read directories containing images
img_files1 = dir(strcat(path1,'*.png'));
img_files2 = dir(strcat(path2,'*.png'));
num_of_images = length(img_files1);

%% Initialize variables for odometry
pos = [0;0;0];
Rpos = eye(3);

%% Start Algorithm
for t = 2 : num_of_images
    %% Read Images
    % for time instant t
    I1_l = imread([img_files1(t).folder, '/', img_files1(t - 1).name]);
    I1_r = imread([img_files2(t).folder, '/', img_files2(t - 1).name]);
    % for time instant t + 1
    I2_l = imread([img_files1(t+1).folder, '/', img_files1(t).name]);
    I2_r = imread([img_files2(t+1).folder, '/', img_files2(t).name]);

    %% Implement SOFT for time instant t+1
    tic;
    [R, tr] = visualSOFT(t, I1_l , I1_r, I2_l, I2_r, P1, P2);
    toc

    %% Estimated pose relative to global frame at t = 0
    pos = pos + Rpos * tr;
    Rpos = R * Rpos;

    %% Plot the odometry transformed data
    subplot(2, 2 , [2 4]);
    scatter(pos(1), pos(3), 'b', 'filled');
    hold on;
    title(sprintf('Odometry plot at frame %d', t))
    xlabel('x-axis (in meters)');
    ylabel('z-axis (in meters)');
    %% Read ground truth pose if flag is true
    if show_gt_flag
      pos_gt = ground_truth(t, :);
      scatter(pos_gt(1), pos_gt(2), 'r', 'filled');
      legend('Estimated Pose', 'Ground Truth Pose')
    end
    %% Pause to visualize the plot
    pause(0.0001);
end
