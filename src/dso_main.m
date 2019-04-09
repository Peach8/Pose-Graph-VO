% build vector of frame structs containing point cloud param and others

% loop through each frame

%   front-end
%   back-end
%   save pose
% end loop

% ============================================
% WINDOW STRUCT
% ============================================

global window
max_num_keyframes = 7;

% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
% Specify the maximum index of frames
window.maxFrameIdx = 1;
% Dictionary of active points and their locations in each frame
%   KeyType - index of active point
%   ValueType - [(x_1, y_1), (x_2, y_2), ...] (location of active point
%       in each frame, where (x_n, y_n) is the location of the active point
%       in frame n. If (x, y) = (-1, 1), then active point not
%       observable in the frame.
window.activePoints = containers.Map('KeyType', 'int32', 'ValueType', 'Any');
% Array of transformation matrices'
window.transform = cell(max_num_keyframes, max_num_keyframes);

% ============================================
% FRAME STRUCT (an element of window)
% ============================================

% Create elements for every frame
num_candidate_points = 2000;
for i=1:max_num_keyframes
    % Storing candidate points in a cell array. The cell array must be
    % ordered by minimum distance to active points (as mentioned in
    % the Google Doc). Each element of this array is a 1 x 2 double
    % array [x_1, y_1], where (x_1, y_1) is the x, y position of the
    % candidate point in the frame.
    window.keyframes{i}.candidatePoints = cell(1, num_candidate_points);
    % distanceToPrevious will hold the frame's distance to
    % the previous frame
    window.keyframes{i}.distanceToPrevious = 0;
end

% ============================================

% feeding frame by frame
for i=1:num_frames
    %decide if a frame could be a keyframe  
    if i == 1
        window.keyframes{window.maxFrameIdx}.candidatePoints = ...
            findCandidatePoints(uw_livingRoomData{i}.Location);
        window.maxFrameIdx = window.maxFrameIdx +1;
    elseif i == 2
        window.keyframes{window.maxFrameIdx}.candidatePoints = ...
            findCandidatePoints(uw_livingRoomData{i}.Location);
        window.keyframes{window.maxFrameIdx}.distanceToPrevious = ...
            distanceBetweenFrames(window.maxFrameIdx + 1,...
                                  window.maxFrameIdx); % will be implemented
        window.maxFrameIdx = window.maxFrameIdx + 1;
    else
        if isKeyFrame(frames{i}) %need to be implemented
            window.keyframes{window.maxFrameIdx}.candidatePoints = ...
                findCandidatePoints(uw_livingRoomData{i}.Location);
            window.keyframes{window.maxFrameIdx}.distanceToPrevious = ...
                distanceBetweenFrames(window.maxFrameIdx + 1,...
                                      window.maxFrameIdx);
            window.maxFrameIdx = window.maxFrameIdx +1;
        end
    end
  
    % calculate transformation from each keyframe to all other keyframes
    for j = 1: window.maxFrameIdx
        % make rkhs registration object
        for k = (j+1) : window.maxFrameIdx
            dvo = rgbd_dvo();
            dvo.set_ptclouds(window.keyFrame{k}, window.keyFrame{j});
            dvo.align();
            window.transform{j,k} = dvo.tform; % Store transformation matrices between frames
        end
    end
    
  % choose candidate points
  
  % send to back end to optimize
  
  % choose keyframe to marginalzie
end
% compare poses to GT and plot results