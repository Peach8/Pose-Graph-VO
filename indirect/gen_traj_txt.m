% gen timestamp for all data

% clc; clear; close all

assco = import_assoc_file('assoc.txt');
rgbName = assco(:,1);
timestamp = zeros(round(length(rgbName)/10-1),1);

for i = 1:length(timestamp)
    timestamp(i) = rgbName(i*10);
end

timestamp = timestamp(1:49);

%% 

fileID = fopen('indirect_traj_woloop_50frame.txt','w');
for i = 1:49
    fprintf(fileID,'%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n',...
        timestamp(i),coords{i}(1),coords{i}(2),coords{i}(3),coords{i}(4),coords{i}(5),coords{i}(6),coords{i}(7));
%     fprintf(fileID,'%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n',...
%         timestamp(i),COORD{i}(1),COORD{i}(2),COORD{i}(3),COORD{i}(4),COORD{i}(5),COORD{i}(6),COORD{i}(7));
end
fclose(fileID);

