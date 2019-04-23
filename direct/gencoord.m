poses = poses(~cellfun('isempty',poses));

coords = cell(length(poses), 1);
for i = 1:length(poses)
    coords{i} = pose_to_coord(poses{i});
end
