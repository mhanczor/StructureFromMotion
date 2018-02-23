function SFM( imPaths, Ks )
%SFM Generates 3D model given a folder of images of the model.
%   Extreme WIP, just something to help us get started.
%   Probably also take in Camera Parameters
%   imagesPaths - cell array of image paths (nx1)
%   Ks - cell array of camera intrinsic matrices (each 3x3) (nx1)

%% User Parameters
% Used
imPaths = imPaths(1:3); %remove this later

matchThresh = 1.5; %default value 1.5

%% Setup the VLFeat Toolbox
VLFEATROOT = fullfile('..', 'vlfeat-0.9.20');
run(fullfile(VLFEATROOT, 'toolbox', 'vl_setup'));
fprintf('VLFeat Version: %s.\n', vl_version);

%% Get image file names and declare variables
nImages = length(imPaths);
folderPath = fileparts(imPaths{1});

%% SIFT
SIFTFilename = fullfile(folderPath, 'SIFT.mat');

%load precomputed SIFT data exists
if exist(SIFTFilename, 'file')==2
    load(SIFTFilename, 'features', 'descriptors', 'matches', 'scores');
else %Compute SIFT data
    [features, descriptors, matches, scores] = SIFT(imPaths, matchThresh);
    %Save Results
    save(SIFTFilename, ...
        'features', 'descriptors', 'matches', 'scores');
end

%% Initial 2 view stereo
pts1 = features{1}(1:2,matches{1,2}(1,:))';
pts2 = features{2}(1:2,matches{1,2}(2,:))';

im1 = imread(imPaths{1});
im2 = imread(imPaths{2});

M = size(im1);
M = max(M(1:2));

figure;
showMatchedFeatures(im1, im2, ...
    pts1, pts2);

%%

[ F, bestInlierIdx ] = ransacF( pts1, pts2, M );
[ E ] = essentialMatrix( F, Ks{1}, Ks{2} );

matches{1,2}(:, ~bestInlierIdx) = []; %delete bad matches
pts1 = features{1}(1:2,matches{1,2}(1,:))';
pts2 = features{2}(1:2,matches{1,2}(2,:))';

M1 = Ks{1} * eye(3,4);
M2s = camera2(E, eye(3), zeros(3,1));
for i = 1:4
    M2 = Ks{2}*M2s(:,:,i);
    [ P, error ] = triangulate( M1, pts1, M2, pts2 );
    
    %% Check if points are in front of camera and break early if possible
    % check if points are in front of camera
    if any(P(:,3) < 0) == false 
        break;
    elseif i==4
        error('No valid M2 !?!');
    end
end
[ P, error ] = triangulate( M1, pts1, M2, pts2, true);

%% Add images iteratively


%% Bundle Adjustment

%% Final Reconstruction.
end

