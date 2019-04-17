% processGT
gtcoord = load('gtcoord.mat');
gtcoord = gtcoord.gtcoord;
gtpose = cell(length(gtcoord),1);
for i = 1:length(gtcoord)
    gtpose{i} = coord_to_pose(gtcoord(i,:));
end

off = gtpose{1};
R = off(1:3,1:3);
t = off(1:3,4);

offset = [R^(-1), -R^(-1)*t;
          zeros(1,3), 1];
xyz = zeros(length(gtpose),3);

for i = 1:length(gtpose)
    gtpose{i} = offset*gtpose{i};
    xyz(i,:) = gtpose{i}(1:3,4)';
end

