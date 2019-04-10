function [pyramid_gray, pyramid_depth, pyramid_intrinsics] = buildPyramid(gray, depth, num_levels)
    pyramid_gray = {};
	pyramid_depth = {};
	pyramid_intrinsics = {};

	current_gray = gray;
	current_depth = depth;
    
    for level = 1:num_levels
		pyramid_gray = {pyramid_gray, current_gray};
		pyramid_depth = {pyramid_depth, current_depth};
		if level < num_levels-1
			current_gray = downsampleGray(current_gray);
			current_depth = downsampleDepth(current_depth);
        end
    end