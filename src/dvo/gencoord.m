coords = cell(49, 1);
for i = 1:49
    coords{i} = pose_to_coord(poses{i});
end