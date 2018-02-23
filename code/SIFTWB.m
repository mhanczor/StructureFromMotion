function [ features, descriptors, matches, scores ] = ...
            SIFTWB( imPaths, matchThresh )
%SIFT Performs SIFT on images

%% Input Checking
if nargin<2
    matchThresh = 1.5;
end

peakThresh = 3;

%% Variable Declaration
nImages = length(imPaths);

features = cell(nImages,1);
descriptors = cell(nImages,1);
matches = cell(nImages);
scores = cell(nImages);

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
for iIm1 = 1:nImages
    for iIm2 = iIm1+1:nImages
%         if iIm1==iIm2
%             continue;
%         end
        [matches{iIm1, iIm2}, scores{iIm1, iIm2}] = ...
                vl_ubcmatch(descriptors{iIm1}, ...
                            descriptors{iIm2}, ...
                            matchThresh) ;
    end
end

end

