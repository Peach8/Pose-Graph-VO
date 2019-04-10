clc; clear; close all
% tic;

% %% Register Two Point Clouds
% dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
% load(dataFile);
% 
% ptCloudRef = ptcloud_edge_filter(livingRoomData{1});
% ptCloudCurrent = ptcloud_edge_filter(livingRoomData{2});
% 
% % ptCloudRef = livingRoomData{1};
% % ptCloudCurrent = livingRoomData{2};


%  load another sample
% example folder
exp_folder = 'examples';

% Kinect intrinsic parameters
K = dlmread(fullfile(exp_folder, 'kinect_params.txt'));

% load (image, depth) pairs and ground truth relative pose
% case_folder = 'case_1';
case_folder = 'case_freibrug2';

% load fixedious image and depth pairs
img_fixed = (imread(fullfile(exp_folder, case_folder, 'img_prev.png')));
dep_fixed = (imread(fullfile(exp_folder, case_folder, 'dep_prev.png')));
dep_fixed = double(dep_fixed)/5000;
dep_fixed(dep_fixed==0) = nan;
% dep_fixed = uint8((dep_fixed / (2^8) -1));
% load movingent image
img_moving = (imread(fullfile(exp_folder, case_folder, 'img_curr.png')));
dep_moving = (imread(fullfile(exp_folder, case_folder, 'dep_curr.png')));
dep_moving = double(dep_moving)/5000;
dep_moving(dep_moving==0) = nan;

% down sample by pyramid
% while size(img_fixed, 1) > 300
%     img_fixed = coarse_to_fine(img_fixed, size(img_fixed, 1), size(img_fixed, 2));
%     dep_fixed = coarse_to_fine(dep_fixed, size(dep_fixed, 1), size(dep_fixed, 2));
%     img_moving = coarse_to_fine(img_moving, size(img_moving, 1), size(img_moving, 2));
%     dep_moving = coarse_to_fine(dep_moving, size(dep_moving, 1), size(dep_moving, 2));
% end

% % convert the image and depth data
% img_fixed = rgb2gray(img_fixed); img_moving = rgb2gray(img_moving);
% dep_fixed = dep_fixed/5000; dep_moving = dep_moving/5000;

[location_fixed, color_fixed] = read_rgbd(img_fixed, dep_fixed);
[location_moving, color_moving] = read_rgbd(img_moving, dep_moving);

% ptCloudRef.Location = location_fixed;
ptCloudRef = pointCloud(location_fixed);
ptCloudRef.Color = uint8(color_fixed);

% ptCloudCurrent.Location = location_moving;
ptCloudCurrent = pointCloud(location_moving);
ptCloudCurrent.Color = uint8(color_moving);


% figure
% subplot(1,2,1)
% pcshow(ptCloudCurrent);
% subplot(1,2,2)
% pcshow(ptCloudRef);
%% downsample point clouds using grids

gridSize = 0.02;
for i = 0.01:-0.001:0.009
    fixed = [];
    fixed.ptcloud = pcdownsample(ptCloudRef, 'gridAverage', i);       
    % load RGB image
%     fixed.image = rgb2gray(livingRoomData{1}.Color);
    fixed.image = rgb2gray(img_fixed);

    % downsample point clouds using grids
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', i);

    % make rkhs registration object
    tic
    dvo = rgbd_dvo();
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
figure
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
%     % Use fixedious moving point cloud as reference.
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
%     % Transform the movingent point cloud to the reference coordinate system
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