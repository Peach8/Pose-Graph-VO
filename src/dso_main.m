% build vector of frame structs containing point cloud param and others

% loop through each frame

%   front-end
%   back-end
%   save pose
% end loop
global window
max_num_keyframes = 7;

% Holds a cell array of frame structs
window.keyframes = cell(1, max_num_keyframes);
% Specify the maximum index of frames
window.maxFrameIdx = 1;
window.transform = cell(max_num_keyframes,max_num_keyframes);

global frames
num_frames = 20;
frames = cell(1, num_frames);

for i=1:num_frames
    % RGB values
    frames{i}.Color = uw_livingRoomData{i}.Color;
    % XYZ values
    frames{i}.ptclound = findCandidatePoints(uw_livingRoomData{i}.Location);% need to be implemented
end

% feedin frame by frame
for i=1:num_frames
  %decide if a frame could be a keyframe  
  if (i==1 || i==2)
      window.keyframes{window.maxFrameIdx} = frames{i};
      window.maxFrameIdx = window.maxFrameIdx +1;
      continue;
  end
  
  if isKeyFrame(frames{i})%need to be implemented
      window.keyframes{window.maxFrameIdx} = frames{i};
      window.maxFrameIdx = window.maxFrameIdx +1;
  else 
      continue;
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