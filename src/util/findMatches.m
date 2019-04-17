function [loc_frame, loc_keyframe, features, points] = findMatches(...
                features, points, frame_idx, kframe_idx_in_window)
    global window;
    global freiburg2;
    
    loc_frame = round(points.Location);
    
    ptcloud_frame = zeros(size(points.Location, 1), 3);
    for i=1:size(points, 1)
        ptcloud_frame(i, :) = freiburg2{frame_idx}.Location(...
            loc_frame(i, 2), loc_frame(i, 1), :);
    end
    rows_nan_frame = any(isnan(ptcloud_frame'));
    features = features(~rows_nan_frame, :);
    points = points(~rows_nan_frame, :);
    
    keyframe_features = ...
        window.keyframes{kframe_idx_in_window}.candidatePoints.features;
    keyframe_points = ...
        window.keyframes{kframe_idx_in_window}.candidatePoints.points;
    % Match features between current frame and latest keyframe
    index_pair = matchFeatures(features, keyframe_features);
    matchedPoints_frame = points(index_pair(:, 1));
    matchedPoints_keyframe = keyframe_points(index_pair(:, 2));
    
    lastwarn(''); % clear last warning
    [~, in_dist, in_orig, status] = estimateGeometricTransform(...
            matchedPoints_frame, matchedPoints_keyframe, 'projective');
    [~, msgid] = lastwarn;
    
    if status == 0 % no error
        % check for warning
        if strcmp(msgid, 'vision:ransac:maxTrialsReached')
            loc_frame = [];
            loc_keyframe = []; 
        else % if no error or warning
            % Determine the x, y coordinates of all matched points and round
            % to convert to pixel location
            loc_frame = round(in_dist.Location);
            loc_keyframe = round(in_orig.Location);
        end
    else % error: notEnoughMatchedPts or notEnoughInlierMatches
        loc_frame = [];
        loc_keyframe = [];        
    end
    

end