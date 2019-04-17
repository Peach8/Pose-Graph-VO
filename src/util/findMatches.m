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
    
    try
        [~, in_dist, in_orig] = estimateGeometricTransform(...
            matchedPoints_frame, matchedPoints_keyframe, 'projective');
        % Determine the x, y coordinates of all matched points and round
        % to convert to pixel location
        loc_frame = round(in_dist.Location);
        loc_keyframe = round(in_orig.Location);
    catch err
        if strcmp(err.identifier, 'vision:points:notEnoughInlierMatches') || strcmp(err.identifier, 'vision:points:notEnoughMatchedPts')
            loc_frame = [];
            loc_keyframe = [];
        end
    end

end