clc, clear, close all;

rgbFiles = dir('./according_to_loadData/rgb/*.png');
depthFiles = dir('./according_to_loadData/depth/*.png');

numFrames = 3; % num frames used in simulation
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
    rgb = imread(strcat('./according_to_loadData/rgb/', rgbFiles(i).name));
    depth = double(imread(strcat('./according_to_loadData/depth/', depthFiles(i).name)));
    depth(depth == 0) = nan;
    
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

