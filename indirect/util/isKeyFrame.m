function [isKey] = isKeyFrame(frame_idx)
    global window;
    global freiburg2;
    
    % Get SURF features and points
    [features, points] = findCandidatePoints(freiburg2{frame_idx}.Color);
    
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

    % Compute initial transformation matrix between point clouds
    % from keyframe to frame
    tform = findInitialTform(ptcloud_frame, ptcloud_keyframe);

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
                tform = findInitialTform(ptcloud_frame, ptcloud_keyframe);
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
    