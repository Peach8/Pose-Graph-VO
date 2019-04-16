clear; close all



%% load point cloud
freiburg2 = load('freiburg2.mat');
freiburg2 = freiburg2.freiburg2;
% load key information
%%%%% to do: KEYS
KEYS = cell(2,1);
KEYS{1} = cell(6,1);
KEYS{1}{1}(1) = 1;
KEYS{1}{1}(2) = 2;
KEYS{1}{2}(1) = 2;
KEYS{1}{2}(2) = 3;
KEYS{1}{3}(1) = 3;
KEYS{1}{3}(2) = 4;
KEYS{1}{4}(1) = 1;
KEYS{1}{4}(2) = 3;
KEYS{1}{5}(1) = 2;
KEYS{1}{5}(2) = 4;
KEYS{1}{6}(1) = 1;
KEYS{1}{6}(2) = 4;


KEYS{2} = cell(4,1);
KEYS{2}{1}(1) = 2;
KEYS{2}{1}(2) = 3;
KEYS{2}{2}(1) = 3;
KEYS{2}{2}(2) = 4;
KEYS{2}{3}(1) = 4;
KEYS{2}{3}(2) = 5;
KEYS{2}{4}(1) = 2;
KEYS{2}{4}(2) = 5;


%%%%% to do: N = window size
N = 4;

% init 
POSE = cell(length(freiburg2),1);
POSE{1} = eye(4);
COORD = cell(length(freiburg2),1);
COORD{1} = pose_to_coord(POSE{1});

for i = 1:length(KEYS)
    
    tic
    
    Z = cell(length(KEYS{i}),1);
    
    beg_idx = KEYS{i}{1}(1);
    end_idx = beg_idx + N - 1;
    
    ptCloudbeg = freiburg2{beg_idx};
    ptCloudend = freiburg2{end_idx};
    
    for j = 1:length(KEYS{i})
        R_init = eye(3);
        T_init = zeros(3,1);
        tform12 = affine3d([R_init, T_init; 0, 0, 0, 1]');
        
        fidx1 = KEYS{i}{j}(1);
        fidx2 = KEYS{i}{j}(2);

        ptCloud1 = freiburg2{fidx1};
        ptCloud2 = freiburg2{fidx2};
        
        % downsample point clouds
        percen_range = 0.04:0.02:0.08;
        if ~isempty(POSE{fidx2})
            disp('Already Calculated')
            tform12.T = (POSE{fidx1}\POSE{fidx2})';
            Z{j} = tform12.T';
            fprintf('%d-%d Transformation Estimate:\n',fidx1,fidx2)
            disp(tform12.T')
        else
            for percen = percen_range
                fprintf('%d-%d\n',fidx1,fidx2)
                fprintf('%f resolution\n',percen)
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
                fprintf('%d-%d Transformation Estimate:\n',fidx1,fidx2)
                disp(tform12.T')
            end
            Z{j} = tform12.T';
        end
    end
    
    % joint optimization 
    fprintf('%d-%d Joint Optimization Transformation\n',beg_idx,end_idx)
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
    figure(1)
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

% %% trajectory
% temp = 357;
% X = zeros(temp,1);
% Y = zeros(temp,1);
% Z = zeros(temp,1);
% 
% 
% for i = 1:temp
% %     coor = pose_to_coord(POSEL{i})
% %     X(i) = coor(1);
% %     Y(i) = coor(2);
% %     Z(i) = coor(3);
%     X(i) = COORD{i}(1);
%     Y(i) = COORD{i}(2);
%     Z(i) = COORD{i}(3);
% end
% 
% 
% 
% A = [X, Y, Z];
% GT = load('GT.mat');
% xyz = GT.xyz;
% xyz = xyz*10;
% 
% % xyz(:,1) = xyz(:,1) - xyz(1,1);
% % xyz(:,2) = xyz(:,2) - xyz(1,2);
% % xyz(:,3) = xyz(:,3) - xyz(1,3);
% 
% 
% 
% figure(4)
% % plot3(X,Y,Z)
% % hold on
% plot3(xyz(:,1),xyz(:,2),xyz(:,3))
% % hold off
% legend('direct','gt')
% 
% 
% 
% %% Stitch a Sequence of Point Clouds
% 
% PtCloud1 = freiburg2{1};
% for i = 2:100
% %     ptCloudCurrent = ptcloud_edge_filter(freiburg2{i});
%     ptCloud2 = freiburg2{i};
%     
%     tform21 = affine3d(POSE{i}^(-1)'); % direct
% %     tform21 = affine3d(poses{i}^(-1)'); % indirect
%     
%     ptCloudtransformed2 = pctransform(ptCloud2, tform21);
% 
%     mergeSize = 0.015;
%     PtCloud1 = pcmerge(PtCloud1, ptCloudtransformed2, mergeSize);
% 
% end
% 
% % Visualize the world scene.
% figure(5)
% pcshow(PtCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% drawnow