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
max_num_keyframes = 3;

% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
window.maxFrameIdx = 0;
% Array of transformation matrices
%   - Index 1 -> matrix between 1 and 2
%   - Index 2 -> matrix between 2 and 3
%   - Index 3 -> matrix between 1 and 3
window.transform = cell(1, max_num_keyframes);
window.flowThresh = 50;

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
num_frames = 20;
for i=1:num_frames
    %decide if a frame could be a keyframe  
    if i == 1 
        % findCandidatePoints -> SIFT
        window.maxFrameIdx = window.maxFrameIdx + 1;
        [feat, points] = findCandidatePoints(freiburg2{i}.Color);
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
        window.transform{3} = window.transform{1} * window.transform{2};
        back_end();
        window.keyframes(1:2) = window.keyframes(2:3);
        window.maxFrameIdx = window.maxFrameIdx - 1;
        window.transform{1} = window.transform{window.maxFrameIdx};
    end

end
% compare poses to GT and plot results