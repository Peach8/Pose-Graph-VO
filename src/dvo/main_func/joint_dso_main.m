%% load point cloud
clear; close all

% % load key information
% kframe_idx = load('window_40_kframe_indices.mat');
% kframe_idx = kframe_idx.kframe_indices;
% KEY = load('window_40_keys.mat');
% KEY = KEY.keys;
% 
% KEYS = cell(1,1);
% KEYS{1} = KEY;
% 
% % load and process data
% freiburg2 = load('freiburg2.mat');
% freiburg2 = freiburg2.freiburg2(kframe_idx);
% 



%%
% init 

% N = window size
N = 3;

POSE = cell(length(freiburg31),1);
POSE{1} = eye(4);
COORD = cell(length(freiburg31),1);
COORD{1} = pose_to_coord(POSE{1});

for i = 1:length(KEYS)
    
    tic
    
    Z = cell(length(KEYS{i}),1);
    
    beg_idx = KEYS{i}{1}(1);
    end_idx = beg_idx + N - 1;
    
    ptCloudbeg = freiburg31{beg_idx};
    ptCloudend = freiburg31{end_idx};
    
    for j = 1:length(KEYS{i})

        fidx1 = KEYS{i}{j}(1);
        fidx2 = KEYS{i}{j}(2);
        
        R_init = eye(3);
        T_init = zeros(3,1);
        tform12 = affine3d([R_init, T_init; 0, 0, 0, 1]');

        ptCloud1 = freiburg31{fidx1};
        ptCloud2 = freiburg31{fidx2};
        
        % downsample point clouds
        percen_range = 0.01:0.02:0.03;
        if ~isempty(POSE{fidx2})
            fprintf('%d-%d Already calculated\n',fidx1,fidx2)
            tform12.T = (POSE{fidx1}\POSE{fidx2})';
            Z{j} = tform12.T';
%             fprintf('%d-%d Transformation Estimate:\n',fidx1,fidx2)
%             disp(tform12.T')
        else
            if fidx2 - fidx1 ~= 1
                idx_off = beg_idx-1;
                jtf = eye(4);
                for r = fidx1-idx_off:fidx2-idx_off-1
                    jtf = jtf*Z{r};
                end
                fprintf('%d-%d Use prior tform as init\n',fidx1,fidx2)
                tform12 = affine3d((jtf)');
            end
            for percen = percen_range
                fprintf('%d-%d %f Resolution\n',fidx1,fidx2,percen)
                % moving - fixed : 1-2
                fixed1 = [];
                fixed1.ptcloud = removeInvalidPoints(pcdownsample(ptCloud2, 'random', percen));
                fixed1.image = rgb2gray(ptCloud2.Color);
                moving1 = removeInvalidPoints(pcdownsample(ptCloud1, 'random', percen));
                % make rkhs registration object
                dvo = rgbd_dvo();
                dvo.set_ptclouds(fixed1, moving1);
                dvo.tform = tform12;
                dvo.align();
                tform12 = dvo.tform;
%                 fprintf('%d-%d Transformation Estimate:\n',fidx1,fidx2)
%                 disp(tform12.T')
            end
            Z{j} = tform12.T';
        end
    end
    
    % joint optimization 
    fprintf('%d-%d Joint optimization\n',beg_idx,end_idx)
    jointPose = joint_optimization(Z, KEYS{i}, N);
    for k = beg_idx:end_idx
        POSE{k} = POSE{beg_idx} * jointPose{k-(beg_idx-1)};
        COORD{k} = pose_to_coord(POSE{k});
    end
    
    toc
    
    % transform beg to end
    tformbe = affine3d((POSE{beg_idx}\POSE{end_idx})');
    ptCloudtransformedbeg = pctransform(ptCloudbeg,tformbe);

    mergeSize = 0.015;
    ptCloudScene = pcmerge(ptCloudend, ptCloudtransformedbeg, mergeSize);
    
    
    
    % Visualize the input images.
    figure(3)
    subplot(2,2,1)
    imshow(ptCloudend.Color)
    title('First input image')
    drawnow

    subplot(2,2,3)
    imshow(ptCloudbeg.Color)
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
end

%% trajectory
temp = 40;
Xc = zeros(temp,1);
Yc = zeros(temp,1);
Zc = zeros(temp,1);

% estimated
for i = 1:temp
    Xc(i) = COORD{i}(1);
    Yc(i) = COORD{i}(2);
    Zc(i) = COORD{i}(3);
end

% ground truth
gtcoord_xyz = load('gtcoord_xyz.mat');
xyz = gtcoord_xyz.xyz;


figure(4)
c = linspace(1,10,length(X));
scatter3(Xc,Yc,Zc,10,c,'d')
hold on
% c = linspace(1,10,length(COORD2(:,1)));
% scatter3(COORD2(:,1),COORD2(:,2),COORD2(:,3),10,c,'*')
c = linspace(1,10,length(xyz(:,1)));
scatter3(-xyz(:,1),xyz(:,2),-xyz(:,3),10,c,'filled')
hold off
legend('direct','gt')
xlabel('x')
ylabel('y')
zlabel('z')


 %% Stitch a Sequence of Point Clouds

PtCloud1 = freiburg31{1};
for i = 2:100
%     ptCloudCurrent = ptcloud_edge_filter(freiburg2{i});
    ptCloud2 = freiburg31{i};
    
    tform21 = affine3d(POSE{i}^(-1)'); % direct
    
    ptCloudtransformed2 = pctransform(ptCloud2, tform21);

    mergeSize = 0.015;
    PtCloud1 = pcmerge(PtCloud1, ptCloudtransformed2, mergeSize);

end

% Visualize the world scene.
figure(5)
pcshow(PtCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow