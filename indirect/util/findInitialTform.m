function tform=findInitialTform(pc1, pc2)

[~, ~, transform] = procrustes(pc1, pc2, 'scaling', false, 'reflection', false);
trans = mean(transform.c, 1);
tform = [transform.T', trans';
         0 0 0 1];

end