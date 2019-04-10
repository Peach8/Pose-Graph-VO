function [features, points] = findCandidatePoints(rgb_image)
    im = rgb2gray(rgb_image);
    points = detectSURFFeatures(im);
    % extractFeatures returns [features, points]
    [features, points] = extractFeatures(im, points);
end