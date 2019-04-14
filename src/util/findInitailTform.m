function tform=findInitailTform(pc1, pc2)

[~, ~, transform] = procrustes(pc1, pc2);
trans = mean(transform.c, 1);
A = [transform.T', trans';
    0 0 0 1];
tform = affine3d(A');
%{
mupc1 = mean(pc1, 1);
mupc2 = mean(pc2, 1);

pc1bar = pc1-mupc1;
pc2bar = pc2-mupc2;

[U,~,V]=svd(pc1bar'*pc2bar);

R = V*U';
T = (mupc1 - mupc2);

A = [R',    T';
     0 0 0 1];
tform = affine3d(A');
%}

%tform = A;

%figure;
%scatter3(pc1(:, 1), pc1(:, 2), pc1(:, 3));
%figure;
%b = (tform.T)' * [pc2'; ones(1,size(pc2,1))];
%scatter3(b(1, :), b(2, :), b(3, :));
end