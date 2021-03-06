clc, clear, close all;

% Run this in the same directory as the data set root folder: rgbd_dataset_freiburg2_xyz
windows = 0; % make 1 if you're using windows pc
numFrames = 20; % num frames used in simulation

if windows == 1
    rgbFiles = dir('.\rgbd_dataset_freiburg2_xyz\rgb\*.png');
    depthFiles = dir('.\rgbd_dataset_freiburg2_xyz\depth\*.png');   
else    
    rgbFiles = dir('./rgbd_dataset_freiburg2_xyz/rgb/*.png');
    depthFiles = dir('./rgbd_dataset_freiburg2_xyz/depth/*.png');
end

freiburg2 = cell(1,numFrames);

% Freiburg 2 - camera params
fx = 520.9;  % focal length x
fy = 521.0;  % focal length y
cx = 325.1;  % optical center x
cy = 249.7;  % optical center y

scaling_factor = 5000;  % depth scaling factor

% set this flag true if want edge extracted point clouds instead of full
extract_edge = false;

for i = 1:numFrames
    if windows == 1
        rgb = imread(strcat('.\rgbd_dataset_freiburg2_xyz\rgb\', rgbFiles(i).name));

        depth = double(imread(strcat('.\rgbd_dataset_freiburg2_xyz\depth\', depthFiles(i).name)));
        depth(depth == 0) = nan;
    else
        rgb = imread(strcat('./rgbd_dataset_freiburg2_xyz/rgb/', rgbFiles(i).name));

        depth = double(imread(strcat('./rgbd_dataset_freiburg2_xyz/depth/', depthFiles(i).name)));
        depth(depth == 0) = nan;
    end
    
    % compute points xyz
    points = double(rgb);
    U = repmat(0:size(depth,2)-1, size(depth,1), 1);
    V = repmat([0:size(depth,1)-1]', 1, size(depth,2));
    points(:,:,3) = depth / scaling_factor;
    points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx;
    points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy;
    freiburg2{i} = pointCloud(points, 'Color', rgb);
    
end

save('freiburg2.mat', 'freiburg2', '-v7.3');

