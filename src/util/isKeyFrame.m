function [isKey] = isKeyFrame(frame_idx)
    global window;
    global freiburg2;
    
    % Get SURF features and points
    [features, points] = findCandidatePoints(freiburg2{frame_idx}.Color);
    
%     loc_frame = round(points.Location);
%     
%     ptcloud_frame = zeros(size(points.Location, 1), 3);
%     for i=1:size(points, 1)
%         ptcloud_frame(i, :) = freiburg2{frame_idx}.Location(...
%             loc_frame(i, 2), loc_frame(i, 1), :);
%     end
%     rows_nan_frame = any(isnan(ptcloud_frame'));
%     features = features(~rows_nan_frame, :);
%     points = points(~rows_nan_frame, :);
%     
%     keyframe_features = ...
%         window.keyframes{window.maxFrameIdx}.candidatePoints.features;
%     keyframe_points = ...
%         window.keyframes{window.maxFrameIdx}.candidatePoints.points;
%     % Match features between current frame and latest keyframe
%     index_pair = matchFeatures(features, keyframe_features);
%     matchedPoints_frame = points(index_pair(:, 1));
%     matchedPoints_keyframe = keyframe_points(index_pair(:, 2));
%     % Get index of most recent keyframe
%     key_idx = window.keyframes{window.maxFrameIdx}.frameIdx;
%     
%     % MUST UNCOMMENT
%     [~, in_dist, in_orig] = estimateGeometricTransform(...
%         matchedPoints_frame, matchedPoints_keyframe, 'projective');
%     %in_dist = matchedPoints_frame;
%     %in_orig = matchedPoints_keyframe;
%     
%     % Determine the x, y coordinates of all matched points and round
%     % to convert to pixel location
%     loc_frame = round(in_dist.Location);
%     loc_keyframe = round(in_orig.Location);
    
    % Find matches using findMatches function
    [loc_frame, loc_keyframe, features, points] = findMatches(features,...
                        points, frame_idx, window.maxFrameIdx);
    % Construct point cloud using location of matched_points
    ptcloud_frame = zeros(size(loc_frame, 1), 3);
    ptcloud_keyframe = zeros(size(loc_keyframe, 1), 3);
    key_idx = window.keyframes{window.maxFrameIdx}.frameIdx;
 
    for i=1:size(loc_frame, 1)
        ptcloud_frame(i, :) = freiburg2{frame_idx}.Location(...
            loc_frame(i, 2), loc_frame(i, 1), :);
        ptcloud_keyframe(i, :) = freiburg2{key_idx}.Location(...
            loc_keyframe(i, 2), loc_keyframe(i, 1), :);
    end
%     rows_nan_frame = any(isnan(ptcloud_frame'));
%     rows_nan_keyframe = any(isnan(ptcloud_keyframe'));
%     rows_remove = rows_nan_frame | rows_nan_keyframe;
%     ptframe = ptcloud_frame(~rows_remove, :);
%     ptkey = ptcloud_keyframe(~rows_remove, :);
    % Compute initial transformation matrix between point clouds
    % from keyframe to frame
    tform = findInitailTform(ptcloud_frame, ptcloud_keyframe);
    
    %a = tform * [ptframe'; ones(1, size(ptframe, 1))];
    %b = [ptkey'; ones(1, size(ptframe, 1))];
    %key_idx = window.keyframes{window.maxFrameIdx}.frameIdx;
    %figure;
    %scatter3(a(1, :), a(2, :), a(3, :));
    %figure;
    %scatter3(b(1, :), b(2, :), b(3, :));
    %c = (b-a).^2;
    
    if size(ptcloud_keyframe, 1) < window.minimum
        window.minimum = size(ptcloud_keyframe, 1);
    end
    flow = 0;
    % Compute flow
    if size(ptcloud_keyframe, 1) >= window.minNumMatches
        flow = computeFlow(ptcloud_keyframe(1:window.minNumMatches, :), ptcloud_frame(1:window.minNumMatches, :));
    end
    % Check if flow is greater than a set threshold
    if flow > window.flowThresh || (size(ptcloud_keyframe, 1) < window.minNumMatches)
        % If flow is greater, add frame to window and store initial guess
        window.maxFrameIdx = window.maxFrameIdx + 1;
        
        % Store transformation matrix
        window.transform{window.maxFrameIdx - 1, window.maxFrameIdx} = tform;
        
        window.keyframes{window.maxFrameIdx}.candidatePoints.features = features;
        window.keyframes{window.maxFrameIdx}.candidatePoints.points = points;
        window.keyframes{window.maxFrameIdx}.frameIdx = frame_idx;
        window.kframe_indices = [window.kframe_indices frame_idx];
        
        % Loop through previous keyframes and find any connections
        for kframe=1:(window.maxFrameIdx - 2)
             [loc_frame, loc_keyframe, ~, ~] = findMatches(features, points, frame_idx,...
                                            kframe);
             % Check if frame share enough points
             if size(loc_frame, 1) >= window.connectionThresh
                % Place one in adjacency matrix
                window.connections(kframe, window.maxFrameIdx) = 1;
                % Get transformation matrix
                % =======
                % Construct point cloud using location of matched_points
                ptcloud_frame = zeros(size(loc_frame, 1), 3);
                ptcloud_keyframe = zeros(size(loc_keyframe, 1), 3);
                key_idx = window.keyframes{kframe}.frameIdx;

                for i=1:size(loc_frame, 1)
                    ptcloud_frame(i, :) = freiburg2{frame_idx}.Location(...
                        loc_frame(i, 2), loc_frame(i, 1), :);
                    ptcloud_keyframe(i, :) = freiburg2{key_idx}.Location(...
                        loc_keyframe(i, 2), loc_keyframe(i, 1), :);
                end

                % Compute initial transformation matrix between point clouds
                % from keyframe to frame
                tform = findInitailTform(ptcloud_frame, ptcloud_keyframe);
                % ======
                % Store transformation matrix
                window.transform{kframe, window.maxFrameIdx} = tform;
             end
        end
                 
        isKey = true;
    else
        isKey = false;
    end
end
    