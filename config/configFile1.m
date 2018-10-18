%%% Configuration File for main
%%% Permits various adjustments to parameters of the Visual Odometry Algorithm

%% Path to the directories containing images
path1 = '../../data/DataSet1/image_00/data/';
path2 = '../../data/DataSet1/image_01/data/';

% Number of images in the dataset
Files = dir(strcat(path1,'*.png'));
NumDataSet = length(Files);

% File name style of images stored in the directories
style = Files(1).name;
style = style(style=='0');