%first download:
% http://vision.middlebury.edu/mview/data/data/dino.zip
% and extract to ../data
%
% This script is used to test multiview reconstruction given imeages with
% known camera calibration matrices K

%% Housekeeping
close all
clc
clear variables

plotEverything = false;
plotLoadMontage = false;
plotFirstSURF = false;
plotFirstValidPoints = false;
plotSubsequentMatches = false;

%% Set Paths (and User Parameters)
matchThresh = 1.5;

imageDir = 'C:\Users\Brian\Desktop\templeRing';
dataImExt = 'png';

% There is one line for each image. The format for each line is:
% "imgname.png k11 k12 k13 k21 k22 k23 k31 k32 k33
%              r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3"
% The projection matrix for that image is K*[R t]
cameraParameterPath = fullfile(imageDir, 'templeR_par.txt');
cameraParameterDelimiter = ' ';
cameraParameterNHeadLines = 1;

%% Load Camera intrinsics
% cameraParameterReadIn is a struct with two fields:
%   data
%   textdata
cameraParameterReadIn = importdata(cameraParameterPath , ...
                                   cameraParameterDelimiter, ...
                                   cameraParameterNHeadLines);
cameraParameterReadIn.textdata(1) = []; %first value is garbage

%% Parse Camera Intrinsics
nImages = size(cameraParameterReadIn.textdata, 1);
Ks = cell(nImages, 1);
Rs = cell(nImages, 1);
Ts = cell(nImages, 1);
for i = 1:nImages
    rowData = cameraParameterReadIn.data(i,:);
    Ks{i} = vec2mat(rowData(1:9), 3);
    Rs{i} = vec2mat(rowData(10:18), 3);
    Ts{i} = vec2mat(rowData(19:end), 1);
end

%% Find all image paths in the dino folder
imPaths = getImPaths(imageDir, dataImExt); %Maybe not use this?



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% PERFORM SFM %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Read the Input Image Sequence
% Read and display the image sequence.

% Use the |imageSet| object to get a list of all image file names in a
% directory.
imSet = imageSet(imageDir);

% Display the images.
if plotEverything || plotLoadMontage
    figure
    montage(imSet.ImageLocation, 'Size', [6, 8]);
end

% Convert the images to grayscale.
images = cell(1, imSet.Count);
for i = 1:imSet.Count
    currIm = read(imSet, i);
    images{i} = rgb2gray(currIm);
end

title('Input Image Sequence');

%% Repackage Camera Parameters
% Very Confident on Ks{i}
% Other aparameters unknwon, like World Points and Accuracy of Estimation
mKs = cat(3,Ks{:});
mRs = cat(3,Rs{:});
mTs = cat(3,Ts{:});

cameraParams = cameraParameters('IntrinsicMatrix', Ks{1}');% ...
%                                 'RotationVectors', ...
%                                 'TranslationVectors', ...);

%% Create a View Set Containing the First View
prevIm = images{1};

border = 50;

% It seems the ROI and other optional parametesr just make it worse in our
% cased
% roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(prevIm);
% prevPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Show the detected SURF Features
if plotEverything || plotFirstSURF
    figure; imshow(prevIm); hold on;
    plot(prevPoints.selectStrongest(100));
end

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
[prevFeatures, prevValidpoints] = extractFeatures(prevIm, prevPoints, ...
                                                  'Upright', false);
if plotEverything || plotFirstValidPoints
    figure; imshow(prevIm); hold on;
    plot(prevValidpoints,'showOrientation',true);
end

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);

%% Add the Rest of the Views
% Go through the rest of the images. For each image
%
% # Match points between the previous and the current image.
% # Estimate the camera pose of the current view relative to the previous
%   view.
% # Compute the camera pose of the current view in the global coordinate 
%   system relative to the first view.
% # Triangulate the initial 3-D world points.
% # Use bundle adjustment to refine all camera poses and the 3-D world
%   points.

for i = 2:5
% for i = 2:numel(images)
    currIm = images{i};
    
    % Detect, extract and match features. OG CODE
%     currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
%     currFeatures = extractFeatures(I, currPoints, 'Upright', true); 

    currPoints = detectSURFFeatures(currIm);
    [currFeatures, currValidpoints] = extractFeatures(currIm, currPoints, ...
                                                  'Upright', false);
                                              
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    if plotEverything || plotSubsequentMatches
        prevImWidth = size(prevIm, 2);
        imPlot = [prevIm, currIm];
        
        loc1 = matchedPoints1.Location;
        loc2 = matchedPoints2.Location;
        
        if i == 2;
            figure
        end
        imshow(imPlot);
        hold on
        for iLine = 1:size(loc1,1)
            line([loc1(iLine,1), loc2(iLine,1) + prevImWidth], ...
                 [loc1(iLine,2), loc2(iLine,2)]);
        end
        title(sprintf('Image %d to %d', i-1, i));
    end
    
    
    % CUSTOM OG PUGH
    % Probably should reject an image around here if the number of
    % correspondances between this and last view are too low
    
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = prevOrientation * relativeOrient;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;
    prevIm = currIm;
end


%% Display Camera Poses
% Display the refined camera poses and 3-D world points.

% Display camera poses.
camPoses = poses(vSet);
figure;
helperPlotCameras(camPoses);

% Exclude noisy 3-D points.
reprojectionErrorThresh = 2; %OG 5
goodIdx = (reprojectionErrors < reprojectionErrorThresh);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on;

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');

%% Compute Dense Reconstruction
% Go through the images again. This time detect a dense set of corners,
% and track them across all views using |vision.PointTracker|.

% Read and undistort the first image
I = undistortImage(images{1}, cameraParams); 

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
    I = undistortImage(images{i}, cameraParams); 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);

%% Display Dense Reconstruction
% Display the camera poses and the dense point cloud.

% Display the refined camera poses.
figure;
helperPlotCameras(camPoses);

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on;

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Dense Reconstruction');

%% References
%
% [1] M.I.A. Lourakis and A.A. Argyros (2009). "SBA: A Software Package for
%     Generic Sparse Bundle Adjustment". ACM Transactions on Mathematical
%     Software (ACM) 36 (1): 1-30.
%
% [2] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
%     Vision," Cambridge University Press, 2003.
%
% [3] B. Triggs; P. McLauchlan; R. Hartley; A. Fitzgibbon (1999). "Bundle
%     Adjustment: A Modern Synthesis". Proceedings of the International
%     Workshop on Vision Algorithms. Springer-Verlag. pp. 298-372.

displayEndOfDemoMessage(mfilename)
