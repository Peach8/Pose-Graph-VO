function flow = computeFlow(tform)
    % tform should be 4x4 SE3 matrix

    global window;

    prevFrame = window.keyframes{window.maxFrameIdx}.Location;
    
    numRows = size(prevFrame,1);
    numCols = size(prevFrame,2);
    
    % build transformed frame and compute flow
    flow = 0;
    for i = 1:numRows % iterate over rows
        for j = 1:numCols % iterate over cols
            prevFrame_xyz = prevFrame(i,j,:);
            
            nextFrame_xyz = tform*[prevFrame_xyz(1), prevFrame_xyz(2), prevFrame_xyz(3), 1]';
            flow = flow + sqrt((prevFrame_xyz - nextFrame_xyz).^2);
%             flow = flow + sqrt((prevFrame_xyz(1) - nextFrame_xyz(1))^2 + ...
%                                (prevFrame_xyz(2) - nextFrame_xyz(2))^2 + ...
%                                (prevFrame_xyz(3) - nextFrame_xyz(3))^3);
        end
    end
    
    flow = flow/(numRows*numCols);
end