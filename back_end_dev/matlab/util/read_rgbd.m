function [location, color] = read_rgbd(rgb, depth)

off = 50;

rgb = rgb(off:end+1-off, off:end+1-off);
depth = depth(off:end+1-off, off:end+1-off);

% depth_ptr = depth(:);
% rgb_ptr = reshape(rgb, [], size(rgb,3));

% fx = 525;  % focal length x
% fy = 525;  % focal length y
% cx = 319.50;  % optical center x
% cy = 239.50;  % optical center y
%             Freiburg 2
            fx = 520.9;  % focal length x
            fy = 521.0;  % focal length y
            cx = 325.1;  % optical center x
            cy = 249.7;  % optical center y
            
H = size(depth, 1)
W = size(depth, 2)

location = zeros(H, W, 3);
color = zeros(H, W, 3);

for y = 1:H
    for x = 1:W
%         off = y*w + x;
%         
%         Z = depth_ptr(off);

        Z = double(depth(y, x));
        X = ((x - cx)* Z / fx );
        Y = ((y - cy)* Z / fy );
        location(y, x, :) = [X, Y, Z];
        color(y, x, :) = rgb(y, x);
    end
end








    
