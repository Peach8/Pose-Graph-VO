function pose = coord_to_pose(coord)
% get 4x4 tf from ROS position and quaterion (x,y,z,w). MATLAB uses
% w,x,y,z order.
ros2pose = @(t,q) ([quat2rotm(q), t; 0 0 0 1]);
t = coord(2:4)';               % position
q = [coord(8),coord(5),coord(6),coord(7)];          % orientation (quaternion)
% get 4x4 tf 
pose = ros2pose(t, q);

end