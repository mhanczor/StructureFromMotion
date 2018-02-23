function SFM_MATLAB( imPaths, Ks )
%SFM Generates 3D model given a folder of images of the model.
%   Extreme WIP, just something to help us get started.
%   Probably also take in Camera Parameters
%   imagesPaths - cell array of image paths (nx1)
%   Ks - cell array of camera intrinsic matrices (each 3x3) (nx1)

%% User Parameters
% Used
matchThresh = 1.5; %default value 1.5

%% Setup the VLFeat Toolbox
VLFEATROOT = fullfile('..', 'vlfeat-0.9.20');
run(fullfile(VLFEATROOT, 'toolbox', 'vl_setup'));
fprintf('VLFeat Version: %s.\n', vl_version);

%% Get image file names and declare variables
nImages = length(imPaths);
folderPath = fileparts(imPaths{1});

%% SIFT
SIFTFilename = fullfile(folderPath, 'SIFT_MATLAB.mat');

%load precomputed SIFT data exists
if exist(SIFTFilename, 'file')==2
    load(SIFTFilename, 'features', 'descriptors', 'matches', 'scores');
else %Compute SIFT data
    [features, descriptors, matches, scores] = SIFT(imPaths, matchThresh);
    %Save Results
    save(SIFTFilename, ...
        'features', 'descriptors', 'matches', 'scores');
end

%% Perfrom Stereo Reconstruction
cameraParams.intrinsicMatrix = Ks{1}';

im1 = imread(imPaths{1});
im2 = imread(imPaths{2});

loc1 = features{1}(1:2,:)';
loc2 = features{2}(1:2,:)';

match12 = matches{1,2}';

matchedPoints1 = loc1(match12(:,1),:);
matchedPoints2 = loc2(match12(:,2),:);

figure;
showMatchedFeatures(im1, im2, ...
    matchedPoints1, matchedPoints2);

%% Estimate the fundamental matrix
[F, epipolarInliers] = estimateFundamentalMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);
end

