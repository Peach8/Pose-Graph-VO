clc; clear; close all

format long

poses = load('poses_50_keyframes_indirect.mat');
poses = poses.poses;
direct_coord = cell(1, 50);
for i=1:50
    direct_coord{i} = pose_to_coord(poses{i});
end

COORD = direct_coord;
%COORD = COORD.COORD;
timestamp = cell(length(COORD),1);

rgbFiles = dir('/home/jdominic/Mich_Courses/W19/masters_f19/rob598_EECS/project/RGB-D_DSO/src/rgbd_dataset_freiburg2_xyz/rgb/*.png');

for i = 1:length(timestamp)
    name = rgbFiles(i*10).name;
    timestamp{i} = str2num(name(1:end-6));
end

fileID = fopen('indirect_50_keyframes.txt','w');
for i = 1:length(COORD)
    fprintf(fileID,'%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n',...
        timestamp{i},COORD{i}(1),COORD{i}(2),COORD{i}(3),COORD{i}(4),COORD{i}(5),COORD{i}(6),COORD{i}(7));
end
fclose(fileID);

