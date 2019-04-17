%% Stitch Point Clouds

n = 40; % window size

PtCloud1 = freiburg2{1};
for i = 2:n
%     ptCloudCurrent = ptcloud_edge_filter(freiburg2{i});
    ptCloud2 = freiburg2{i};
    
    tform21 = affine3d(poses2{i}^(-1)');
    
    ptCloudtransformed2 = pctransform(ptCloud2, tform21);

    mergeSize = 0.015;
    PtCloud1 = pcmerge(PtCloud1, ptCloudtransformed2, mergeSize);

end

% Visualize the world scene.
figure(5)
pcshow(PtCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow