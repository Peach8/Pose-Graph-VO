%% load data
% Demo: 30-batch test
clear; close all

% load keyframe information
% In 30-batch test, kframe indices are the first 30, so we don't need it 
% kframe_idx = load('kframe_indices.mat');

% load keys
KEY = load('./test/keys.mat');
KEY = KEY.keys;
KEYS{1} = KEY;

% % load and process data
freiburg2 = load('./test/freiburg2.mat');
freiburg2 = freiburg2.freiburg;


%%
% init 

% N = window size
N = 30;

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

        fidx1 = KEYS{i}{j}(1);
        fidx2 = KEYS{i}{j}(2);
        
        R_init = eye(3);
        T_init = zeros(3,1);
        tform12 = affine3d([R_init, T_init; 0, 0, 0, 1]');

        ptCloud1 = freiburg2{fidx1};
        ptCloud2 = freiburg2{fidx2};
        
        % downsample point clouds
        percen_range = 0.04:0.02:0.08;
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
    
%     % transform beg to end
%     tformbe = affine3d((POSE{beg_idx}\POSE{end_idx})');
%     ptCloudtransformedbeg = pctransform(ptCloudbeg,tformbe);
% 
%     mergeSize = 0.015;
%     ptCloudScene = pcmerge(ptCloudend, ptCloudtransformedbeg, mergeSize);
    
    
    
%     % Visualize the input images.
%     figure(3)
%     subplot(2,2,1)
%     imshow(ptCloudend.Color)
%     title('First input image')
%     drawnow
% 
%     subplot(2,2,3)
%     imshow(ptCloudbeg.Color)
%     title('Second input image')
%     drawnow
% 
%     % Visualize the world scene.
%     subplot(2,2,[2,4])
%     pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
%     title('Initial world scene')
%     xlabel('X (m)')
%     ylabel('Y (m)')
%     zlabel('Z (m)')
%     drawnow
end






