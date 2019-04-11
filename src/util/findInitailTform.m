function tform=findInitailTform(pc1, pc2)

mupc1 = mean(pc1, 1);
mupc2 = mean(pc2, 1);

pc1bar = pc1-mupc1;
pc2bar = pc2-mupc2;

[U,~,V]=svd(pc1bar'*pc2bar);

R = V*U';
T = (mupc2 - mupc1)';
A = [R,     T;
     0 0 0 1];
%tform = affine3d(A);
tform = A;
end