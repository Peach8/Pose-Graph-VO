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
max_num_keyframes = 3;
window.num_keyframes = 0;
% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
window.maxFrameIdx = 0;
% Array of transformation matrices
%   - Index 1 -> matrix between 1 and 2
%   - Index 2 -> matrix between 2 and 3
%   - Index 3 -> matrix between 1 and 3
window.transform = cell(1, max_num_keyframes);
% Threshold must be < 0.5647 (0.3 works well)
window.flowThresh = 0.1;
% There must be at least 20 non-outlier matches
% between keyframes
window.minNumMatches = 120;
window.minimum = 400;

% ============================================
% FRAME STRUCT (an element of window)
% Properties:
%   - candidatePoints (array of point locations)
%   - frameIdx (index of frame for accessing point cloud)
% ============================================

% feeding frame by frame
global freiburg2;
% freiburg2 = load('freiburg2.mat');
% freiburg2 = freiburg2.freiburg2;
num_frames = 500;
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
        window.num_keyframes = window.num_keyframes + 1;
        features = window.keyframes{window.maxFrameIdx}.candidatePoints.features;
        points = window.keyframes{window.maxFrameIdx}.candidatePoints.points;
        keyframe_features = window.keyframes{window.maxFrameIdx - 2}.candidatePoints.features;
        keyframe_points = window.keyframes{window.maxFrameIdx - 2}.candidatePoints.points;
        index_pair = matchFeatures(features, keyframe_features);
        matchedPoints_frame = points(index_pair(:, 1));
        matchedPoints_keyframe = keyframe_points(index_pair(:, 2));
        frame_idx = window.keyframes{window.maxFrameIdx}.frameIdx;
        key_idx = window.keyframes{window.maxFrameIdx - 2}.frameIdx;
      
        [~, in_dist, in_orig] = estimateGeometricTransform(...
            matchedPoints_frame, matchedPoints_keyframe, 'similarity');
        
        loc_frame = round(in_dist.Location);
        loc_keyframe = round(in_orig.Location);
        % Construct point cloud using location of matched_points
        ptcloud_frame = zeros(size(loc_frame, 1), 3);
        ptcloud_keyframe = zeros(size(loc_keyframe, 1), 3);
   
        for j=1:size(loc_frame, 1)
            ptcloud_frame(j, :) = freiburg2{frame_idx}.Location(...
                loc_frame(j, 2), loc_frame(j, 1), :);
            ptcloud_keyframe(j, :) = freiburg2{key_idx}.Location(...
                loc_keyframe(j, 2), loc_keyframe(j, 1), :);
        end
        
        rows_nan_frame = any(isnan(ptcloud_frame'));
        rows_nan_keyframe = any(isnan(ptcloud_keyframe'));
        rows_remove = rows_nan_frame | rows_nan_keyframe;
        ptframe = ptcloud_frame(~rows_remove, :);
        ptkey = ptcloud_keyframe(~rows_remove, :);
        % Compute initial transformation matrix between point clouds
        % from keyframe to frame
        tform = findInitailTform(ptframe, ptkey);
        window.transform{3} = tform;
        
        det(window.transform{1})
        det(window.transform{2})
        det(window.transform{3})
        
        % Run joint optimization
        poseGraph = cell(1, max_num_keyframes);
        poseGraph{1} = [1, 2]; poseGraph{2} = [2, 3]; poseGraph{3} = [1, 3];
        optimized_tforms = joint_optimization(window.transform,  poseGraph);
        
        % Place transforms from 1-2 and 2-3 in poses
        poses{counter + 1} = poses{counter} * optimized_tforms{1};
        poses{counter + 2} = poses{counter + 1} * optimized_tforms{2};
        counter = counter + 1;
        
        window.transform = optimized_tforms;
        window.keyframes(1:2) = window.keyframes(2:3);
        window.maxFrameIdx = window.maxFrameIdx - 1;
        window.transform{1} = window.transform{window.maxFrameIdx};
    end

end
% compare poses to GT and plot results