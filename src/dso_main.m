% build vector of frame structs containing point cloud param and others

% loop through each frame

%   front-end
%   back-end
%   save pose
% end loop
global poses;
% ============================================
% WINDOW STRUCT
% ============================================

global window

% This will now be a dynamic variable.
max_num_keyframes = 3;

% There will be 2 keyframes before we first increment num_keyframes
window.num_keyframes = 2;
% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
% Frame indices of all keyframes
window.kframe_indices = [];

window.maxFrameIdx = 0;
% Array of transformation matrices
%   - Index 1 -> matrix between 1 and 2
%   - Index 2 -> matrix between 2 and 3
%   - Index 3 -> matrix between 1 and 3
window.transform = cell(max_num_keyframes, max_num_keyframes);
% Connections will be a binary adjacency matrix that will be converted
% into a 'Keys' variable to pass to joint_optimization
window.connections = zeros(max_num_keyframes, max_num_keyframes);
% Threshold must be < 0.5647 (0.3 works well)
window.flowThresh = 0.2;
% There must be at least 20 non-outlier matches
% between keyframes
%window.minNumMatches = 120;
window.minNumMatches = 400;
window.connectionThresh = 80;

window.minimum = 400;

% ============================================
% FRAME STRUCT (an element of window)
% Properties:
%   - candidatePoints (array of point locations)
%   - frameIdx (index of frame for accessing point cloud)
% ============================================

% feeding frame by frame
global freiburg2;
freiburg2 = load('freiburg2.mat');
freiburg2 = freiburg2.freiburg2;
num_frames = 10;
% The poses array will store all camera poses over the trajectory.
% The first pose will simply be the identity matrix.
poses = cell(1, num_frames);
poses{1} = eye(4);

% Counter to place poses in the right index
counter = 1;
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
%         window.num_keyframes = window.num_keyframes + 1;
%         features = window.keyframes{window.maxFrameIdx}.candidatePoints.features;
%         points = window.keyframes{window.maxFrameIdx}.candidatePoints.points;
%         keyframe_features = window.keyframes{window.maxFrameIdx - 2}.candidatePoints.features;
%         keyframe_points = window.keyframes{window.maxFrameIdx - 2}.candidatePoints.points;
%         index_pair = matchFeatures(features, keyframe_features);
%         matchedPoints_frame = points(index_pair(:, 1));
%         matchedPoints_keyframe = keyframe_points(index_pair(:, 2));
%         frame_idx = window.keyframes{window.maxFrameIdx}.frameIdx;
%         key_idx = window.keyframes{window.maxFrameIdx - 2}.frameIdx;
%       
%         [~, in_dist, in_orig] = estimateGeometricTransform(...
%             matchedPoints_frame, matchedPoints_keyframe, 'projective');
%         
%         loc_frame = round(in_dist.Location);
%         loc_keyframe = round(in_orig.Location);
%         % Construct point cloud using location of matched_points
%         ptcloud_frame = zeros(size(loc_frame, 1), 3);
%         ptcloud_keyframe = zeros(size(loc_keyframe, 1), 3);
%    
%         for j=1:size(loc_frame, 1)
%             ptcloud_frame(j, :) = freiburg2{frame_idx}.Location(...
%                 loc_frame(j, 2), loc_frame(j, 1), :);
%             ptcloud_keyframe(j, :) = freiburg2{key_idx}.Location(...
%                 loc_keyframe(j, 2), loc_keyframe(j, 1), :);
%         end
        %{
        rows_nan_frame = any(isnan(ptcloud_frame'));
        rows_nan_keyframe = any(isnan(ptcloud_keyframe'));
        rows_remove = rows_nan_frame | rows_nan_keyframe;
        ptframe = ptcloud_frame(~rows_remove, :);
        ptkey = ptcloud_keyframe(~rows_remove, :);
        %}
        % Compute initial transformation matrix between point clouds
        % from keyframe to frame
%         tform = findInitailTform(ptcloud_frame, ptcloud_keyframe);
%         window.transform{3} = tform;
%         
%         det(window.transform{1})
%         det(window.transform{2})
%         det(window.transform{3})
%         
%         if det(window.transform{3}) < 0.99
%             display("here")
%         end
        
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
        optimized_tforms = joint_optimization(transform,  keys);
        tform = findInitailTform(ptcloud_frame, ptcloud_keyframe);
        
        % Place transforms from all keyframes in poses
        for n=2:length(optimized_tforms)
            poses{counter + n} = poses{counter} * optimized_tforms{n};
        end
        counter = counter + 1;
        
        % Calculate successive transformations
        window.transform{1,2} = optimized_tforms{1};
        for n=2:length(optimized_tforms)
            window.transform{n,n+1} = optimized_tforms{n} / (optimized_tforms{n-1});
        end
        
        % Recompute the initial guesses for any connected nodes
        for n=1:size(connected_nodes, 1)
            idx1 = connected_nodes(n,1);
            idx2 = connected_nodes(n,2);
            window.transform{idx1, idx2} = window.transform{idx1, idx1 + 1};
            l=idx1 + 1;
            while(l < idx2)
                window.transform{idx1, idx2} = window.transform{l, l+1} * ...
                                                window.transform{idx1, idx2};
                l = l + 1;
            end
        end
       
        % Shift all transformations back one row and back one column, and
        % shift adjacency matrix in the same way
        window.connections = zeros(max_num_keyframes, max_num_keyframes);
        for n = 1:length(keys)
            idx1 = keys{n}(1);
            idx2 = keys{n}(2);
            if (idx1 >= 1 && idx2 >= 1)
                window.transform{idx1-1, idx2-1} = optimized_tforms{n};
                window.connections{idx1-1, idx2-1} = 1;
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
est_traj = zeros(window.num_keyframes, 3);
for j = 1:window.num_keyframes
    pt = poses{j} * [0; 0; 0; 1];
    est_traj(j, :) = pt(1:3)';
end
    
disp("DONE")
% compare poses to GT and plot results