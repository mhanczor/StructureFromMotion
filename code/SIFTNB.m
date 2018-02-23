function [ features, descriptors, matches, scores ] = ...
            SIFTNB( imPaths, matchThresh )
%SIFT Performs Narrow Band SIFT on images
% Matches and Scores are between sequential images.
% i.e. matches{1} are the matches between im1 and im2
%
% ToDo: Possibly enfore pixel radius locality between sequential images.

%% Input Checking
if nargin<2
    matchThresh = 1.5;
end

peakThresh = 3;

%% Variable Declaration
nImages = length(imPaths);

features = cell(nImages, 1);
descriptors = cell(nImages, 1);
matches = cell(nImages-1, 1);
scores = cell(nImages-1, 1);

%% Get Features and Descriptors
for iIm = 1:nImages
    imPath = imPaths{iIm};
    imColor = imread(imPath);
    imGray = single(rgb2gray(imColor));
    
    % The matrix f has a column for each frame.
    % A frame is a disk of center f(1:2), scale f(3) and orientation f(4).
    % descriptors
    [features{iIm}, descriptors{iIm}] = vl_sift(imGray, ...
            'PeakThresh', peakThresh);
end

%% Match Features Between Images
% Can probably do iIm2 = iIm1+1:nImages or something more efficient instead
for i = 1:nImages-1
    [matches{i}, scores{i}] = ...
            vl_ubcmatch(descriptors{i}, ...
                        descriptors{i+1}, ...
                        matchThresh) ;
end

end

