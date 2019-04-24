clc, clear, close all;
global poses;
global window

% This will now be a dynamic variable.
max_num_keyframes = 30;

% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
% Frame indices of all keyframes
window.kframe_indices = [];

window.maxFrameIdx = 0;
window.transform = cell(max_num_keyframes, max_num_keyframes);
% Connections will be a binary adjacency matrix that will be converted
% into a 'Keys' variable to pass to joint_optimization
window.connections = zeros(max_num_keyframes, max_num_keyframes);
window.flowThresh = 0.2;
% There must be at least 20 non-outlier matches
% between keyframes

%window.minNumMatches = 120;
window.minNumMatches = 500;
window.connectionThresh = 80;

window.minimum = 400;

% load dataset
global freiburg2;
freiburg2 = load('freiburg2.mat');
freiburg2 = freiburg2.freiburg;
num_frames = size(freiburg2, 2);
% The poses array will store all camera poses over the trajectory.
% The first pose will simply be the identity matrix.
poses = cell(1, num_frames);
poses{1} = eye(4);

% Counter to place poses in the right index
counter_pose = 1;
for i=1:num_frames
    i
    %decide if a frame could be a keyframe
    if i == 1 
        % findCandidatePoints -> SIFT
        window.maxFrameIdx = window.maxFrameIdx + 1;
        [feat, points] = findCandidatePoints(freiburg2{i}.Color);
        
        loc_frame = round(points.Location);    
        
        ptcloud_frame = zeros(size(points.Location, 1), 3);
        for j=1:size(points, 1)
            ptcloud_frame(j, :) = freiburg2{i}.Location(...
                loc_frame(j, 2), loc_frame(j, 1), :);
        end
        rows_nan_frame = any(isnan(ptcloud_frame'));
        feat = feat(~rows_nan_frame, :);
        points = points(~rows_nan_frame, :);
        
        window.keyframes{window.maxFrameIdx}.candidatePoints.features = feat;
        window.keyframes{window.maxFrameIdx}.candidatePoints.points = points;
        % Store the frame index (to access corresponding point cloud)
        window.keyframes{window.maxFrameIdx}.frameIdx = i;
        window.kframe_indices = [window.kframe_indices i];
        % DEBUG
        %figure;
        %imshow(freiburg2{i}.Color); hold on;
        %scatter(points.Location(:, 1), points.Location(:, 2), 'filled');
    else
        if isKeyFrame(i)
        else
            continue
        end
    end
    
    % If three frames are in window, marginalize the oldest one
    if window.maxFrameIdx == max_num_keyframes
        % Find non-zero elements in adjacency matrix
        G = graph(window.connections, 'upper');
        connected_nodes = G.Edges.EndNodes;
        keys = {};
        transform = {};
        for n = 1:window.maxFrameIdx-1
            keys{n} = [n n+1];
            transform{n} = window.transform{n, n+1};
        end
        counter = 1;
        for m = n + 1:(n + size(connected_nodes, 1))
            keys{m} = connected_nodes(counter, :);
            transform{m} = window.transform{keys{m}(1), keys{m}(2)};
            counter = counter + 1;
        end
        
        % Run joint optimization
        optimized_tforms = joint_optimization(transform,  keys, max_num_keyframes);
        
        % Place transforms from all keyframes in poses
        for n=2:length(optimized_tforms)
            poses{counter_pose + (n-1)} = poses{counter_pose} * optimized_tforms{n};
        end
        counter_pose = counter_pose + 1;
        
        % Calculate successive transformationsm
        window.transform{1,2} = optimized_tforms{2};
        for n=2:(length(optimized_tforms) - 1)
            window.transform{n,n+1} = optimized_tforms{n} \ (optimized_tforms{n+1});
        end
        
        % Recompute the initial guesses for any connected nodes
        for n=1:size(connected_nodes, 1)
            idx1 = connected_nodes(n,1);
            idx2 = connected_nodes(n,2);
            window.transform{idx1, idx2} = window.transform{idx1, idx1 + 1};
            l=idx1 + 1;
            while(l < idx2)
               % window.transform{idx1, idx2} = window.transform{l, l+1} * ...
                %                                window.transform{idx1, idx2};
                window.transform{idx1, idx2} = window.transform{idx1, idx2} * ...
                                                window.transform{l, l+1};                            
                l = l + 1;
            end
        end
       
        % Shift all transformations back one row and back one column, and
        % shift adjacency matrix in the same way
        window.connections = zeros(max_num_keyframes, max_num_keyframes);
        for n = 1:length(keys)
            idx1 = keys{n}(1);
            idx2 = keys{n}(2);
            if (idx1 > 1 && idx2 > 1)
                window.transform{idx1-1, idx2-1} = window.transform{idx1, idx2};
                window.transform{idx1,idx2}=[];
                if (abs((idx1-1) - (idx2-1)) > 1)
                    window.connections(idx1-1, idx2-1) = 1;
                end
            end
        end
        
        % Shift all keyframes back one
        for n=1:(window.maxFrameIdx - 2)
            window.keyframes(n:n+1) = window.keyframes(n+1:n+2);
        end
        
        % Decrease the number of keyframes by decreasing window.maxFrameIdx
        window.maxFrameIdx = window.maxFrameIdx - 1;
    end

end

% Get trajectory
est_traj = zeros(length(window.kframe_indices), 3);
for j = 1:length(window.kframe_indices)
    poses{j}
    pt = poses{j} * [0; 0; 0; 1];
    est_traj(j, :) = pt(1:3)';
end

% save('keys.mat')

disp("DONE")

% Draw Merged Point Clouds
n = length(window.kframe_indices); % window size

PtCloud1 = freiburg2{1};
for i = 2:n
%     ptCloudCurrent = ptcloud_edge_filter(freiburg2{i});
    ptCloud2 = freiburg2{window.kframe_indices(i)};
    
    tform21 = affine3d(poses{i}^(-1)');
    
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
