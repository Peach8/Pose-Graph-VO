function [isKey] = isKeyFrame(frame_idx)
    global window;
    % Get SURF features and points
    [features, points] = findCandidatePoints(freiburg{i}.Color);
    [keyframe_features, keyframe_points] = window.keyframes{window.maxFrameIdx};
    % Match features between current frame and latest keyframe
    index_pair = matchFeatures(features, keyframe_features);
    matchedPoints_frame = points(index_pair(:, 1));
    matchedPoints_keyframe = keyframe_points(index_pair(:, 2));
    % Determine the x, y coordinates of all matched points
    loc_frame = matchedPoints_frame.Location;
    loc_keyframe = matchedPoints_keyframe.Location;
    % Construct point cloud using location of matched_points
    
    % Compute initial transformation matrix between point clouds
    
    % Compute flow
    
    % Check if flow is greater than a set threshold
    
    % If flow is greater, add frame to window and store initial guess
    % in window.transforms
end
    