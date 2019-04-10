function tform=findInitialTform(ptCloud1, ptCloud2)

[m,n,d] = size(ptCloud1);
[e,f,h] = size(ptCloud2);

pc1 = reshape(ptCloud1,m*n,d);
pc2 = reshape(ptCloud2,e*f,h);

mupc1 = mean(pc1, 1);
mupc2 = mean(pc2, 1);

pc1bar = pc1-mupc1;
pc2bar = pc2-mupc2;

[U,~,V]=svd(pc1bar'*pc2bar);

R = V*U';
T = mupc2 - mupc1;
A = [R     T
     0 0 0 1];
tform = affine3d(A);    
end