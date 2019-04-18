clc; clear; close all

format long

COORD = load('direct_coord_win3_stepall.mat');
COORD = COORD.COORD;
timestamp = cell(length(COORD),1);

rgbFiles = dir('C:/Personal Files/Research/dataset/rgbd_dataset_freiburg2_xyz/rgb/*.png');

for i = 1:length(timestamp)
    name = rgbFiles(10*(i-1)+1).name;
    timestamp{i} = str2num(name(1:end-6));
end

fileID = fopen('direct_estimated_traj_win3_stepall.txt','w');
for i = 1:357
    fprintf(fileID,'%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n',...
        timestamp{i},COORD{i}(1),COORD{i}(2),COORD{i}(3),COORD{i}(4),COORD{i}(5),COORD{i}(6),COORD{i}(7));
end
fclose(fileID);

