% clc; clear; close all
% % tic;
% 
% %% Register Two Point Clouds
% dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
% load(dataFile);
% 
% ptCloudRef = ptcloud_edge_filter(livingRoomData{1});
% ptCloudCurrent = ptcloud_edge_filter(livingRoomData{2});
% 
% % ptCloudRef = livingRoomData{1};
% % ptCloudCurrent = livingRoomData{2};


%% rgbd image reading port from maani

clc; clear; close all

% RGB intrinsic calibration parameters
% Freiburg 1
% fx = 517.3;  % focal length x
% fy = 516.5;  % focal length y
% cx = 318.6;  % optical center x
% cy = 255.3;  % optical center y

% Freiburg 2
fx = 520.9;  % focal length x
fy = 521.0;  % focal length y
cx = 325.1;  % optical center x
cy = 249.7;  % optical center y

% Freiburg 3
% fx = 535.4;  % focal length x
% fy = 539.2;  % focal length y
% cx = 320.1;  % optical center x
% cy = 247.6;  % optical center y

scaling_factor = 5000;  % depth scaling factor

% set this flag true if want edge extracted point clouds instead of full
extract_edge = false;

% example folder
exp_folder = 'examples';

% % Kinect intrinsic parameters
% K = dlmread(fullfile(exp_folder, 'kinect_params.txt'));

% load (image, depth) pairs and ground truth relative pose
% case_folder = 'case_1';
case_folder = 'case_freibrug2';


% load RGB image
rgb1 = imread(fullfile(exp_folder, case_folder, 'img_prev.png'));
rgb2 = imread(fullfile(exp_folder, case_folder, 'img_curr.png'));

% load Depth image
depth1 = double(imread(fullfile(exp_folder, case_folder, 'dep_prev.png')));
depth1(depth1 == 0) = nan;
depth2 = double(imread(fullfile(exp_folder, case_folder, 'dep_curr.png')));
depth2(depth2 == 0) = nan;

% compute points xyz
points1 = double(rgb1);
U1 = repmat(0:size(depth1,2)-1, size(depth1,1), 1);
V1 = repmat([0:size(depth1,1)-1]', 1, size(depth1,2));
points1(:,:,3) = depth1 / scaling_factor;
points1(:,:,1) = (U1 - cx) .* points1(:,:,3) ./ fx;
points1(:,:,2) = (V1 - cy) .* points1(:,:,3) ./ fy;
point_cloud1 = pointCloud(points1, 'Color', rgb1);

points2 = double(rgb2);
U2 = repmat(0:size(depth2,2)-1, size(depth2,1), 1);
V2 = repmat([0:size(depth2,1)-1]', 1, size(depth2,2));
points2(:,:,3) = depth2 / scaling_factor;
points2(:,:,1) = (U2 - cx) .* points2(:,:,3) ./ fx;
points2(:,:,2) = (V2 - cy) .* points2(:,:,3) ./ fy;
point_cloud2 = pointCloud(points2, 'Color', rgb2);


if extract_edge
   point_cloud1 = ptcloud_edge_filter(point_cloud1);
   point_cloud2 = ptcloud_edge_filter(point_cloud2);
end

% ptCloudRef.Location = location_fixed;
ptCloudRef = point_cloud1;

% ptCloudCurrent.Location = location_moving;
ptCloudCurrent = point_cloud2;

% 
% figure
% subplot(1,2,1)
% pcshow(ptCloudRef);
% 
% subplot(1,2,2)
% pcshow(ptCloudCurrent);

%%
% testc = removeInvalidPoints(pcdownsample(ptCloudRef, 'random', 0.5));
testc = pcdownsample(ptCloudRef, 'gridAverage', 0.01);
cloud = testc.Location;
U = cloud(:,1) * fx ./ cloud(:,3) + cx;
U = round(U);
V = cloud(:,2) * fy ./ cloud(:,3) + cy;
V = round(V);
image = uint8(zeros(480,640,1));
testrgb = rgb2gray(rgb1);
for i = 1:size(U,1) 
        image(V(i),U(i))= testrgb(V(i),U(i));
%         0.299 * testc.Color(i,1) + 0.587 * testc.Color(i,2) + 0.114 * testc.Color(i,3);
end
figure(8)
imshow(image)
figure(7)
imshow(ptCloudRef.Color)
%% downsample
dvo = rgbd_dvo();
% percen = 0.05;
% for gridSize = 0.04:-0.002:0.009
for percen = 0.05:0.02:0.2
    % downsample point clouds using grids
    fixed = [];
    fixed.ptcloud = removeInvalidPoints(pcdownsample(ptCloudRef, 'random', percen));
%     fixed.ptcloud = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
    % load RGB image
%     fixed.image = rgb2gray(livingRoomData{1}.Color);
    fixed.image = rgb2gray(rgb1);

    % downsample point clouds using grids
%     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    moving = removeInvalidPoints(pcdownsample(ptCloudCurrent, 'random', percen));
    % make rkhs registration object
    tic

    dvo.set_ptclouds(fixed, moving);
    dvo.align();
    tform = dvo.tform;
    disp('RGBD DVO Object Transformation Estimate:')
    disp(tform.T')
    toc
    
end
% ptCloudAligned = pctransform(livingRoomData{2},tform);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
% ptCloudScene = pcmerge(livingRoomData{1}, ptCloudAligned, mergeSize);
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

% Visualize the input images.
figure(2)
subplot(2,2,1)
% imshow(livingRoomData{1}.Color)
imshow(ptCloudRef.Color)
title('First input image')
drawnow

subplot(2,2,3)
% imshow(livingRoomData{2}.Color)
imshow(ptCloudCurrent.Color)
title('Second input image')
drawnow

% Visualize the world scene.
subplot(2,2,[2,4])
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow

%% Stitch a Sequence of Point Clouds
% To compose a larger 3-D scene, repeat the same procedure as above to
% process a sequence of point clouds. Use the first point cloud to
% establish the reference coordinate system. Transform each point cloud to
% the reference coordinate system. This transformation is a multiplication
% of pairwise transformations.

% Store the transformation object that accumulates the transformation.
% accumTform = tform; 
% 
% figure
% hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% title('Updated world scene')
% % Set the axes property for faster rendering
% hAxes.CameraViewAngleMode = 'auto';
% hScatter = hAxes.Children;
% 
% for i = 3:length(livingRoomData)
%     ptCloudCurrent = ptcloud_edge_filter(livingRoomData{i});
%        
%     % Use previous moving point cloud as reference.
%     fixed.ptcloud = moving;
%     % load RGB image
%     fixed.image = rgb2gray(livingRoomData{i}.Color);
%     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
%     
% %     tform = pc_gradient_descent(moving, fixed);
%     dvo.set_ptclouds(fixed, moving);
%     dvo.align();
%     tform = dvo.tform;
% 
%     % Transform the current point cloud to the reference coordinate system
%     % defined by the first point cloud.
%     accumTform = affine3d(tform.T * accumTform.T);
%     ptCloudAligned = pctransform(livingRoomData{i}, accumTform);
%     
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% 
%     % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
% end
% 
% % During the recording, the Kinect was pointing downward. To visualize the
% % result more easily, let's transform the data so that the ground plane is
% % parallel to the X-Z plane.
% angle = -pi/10;
% A = [1,0,0,0;...
%      0, cos(angle), sin(angle), 0; ...
%      0, -sin(angle), cos(angle), 0; ...
%      0 0 0 1];
% ptCloudScene = pctransform(ptCloudScene, affine3d(A));
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
%         'Parent', hAxes)
% title('Updated world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% toc;