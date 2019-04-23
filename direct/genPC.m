%% Stitch a Sequence of Point Clouds
% load('freiburg2.mat')
% freiburg2 = freiburg2(kframeindiceswindow5);
PtCloud1 = freiburg2{1};
for i = 2:80
    
    ptCloud2 = freiburg2{i};
    
    tform21 = affine3d(POSE{i}^(-1)'); % direct
    
    ptCloudtransformed2 = pctransform(ptCloud2, tform21);

    mergeSize = 0.015;
    PtCloud1 = pcmerge(PtCloud1, ptCloudtransformed2, mergeSize);

end

% Visualize the world scene.
figure(5)
pcshow(PtCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('indirect')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow