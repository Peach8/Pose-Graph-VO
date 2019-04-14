function coord = pose_to_coord(pose)
% get quaterion (x,y,z,w) from 4x4  ROS position. MATLAB uses
% w,x,y,z order.
R = pose(1:3,1:3);
t = pose(1:3, 4);

q = rotm2quat(R);
% coord = [tx, ty, tz, qx, qy, qz, qw]
coord = [t(1), t(2), t(3), q(2), q(3), q(4), q(1)];
end