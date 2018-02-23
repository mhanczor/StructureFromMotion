%% Structure From Motion From Multiple Views
% Structure from motion (SfM) is the process of estimating the 3-D structure 
% of a scene from a set of 2-D views. It is used in many applications, such 
% as robot navigation, autonomous driving, and augmented reality. This 
% example shows you how to estimate the poses of a calibrated camera from 
% a sequence of views, and reconstruct the 3-D structure of the scene up to
% an unknown scale factor.

% Copyright 2015 The MathWorks, Inc. 

%% Overview
% This example shows how to reconstruct a 3-D scene from a sequence of 2-D
% views taken with a camera calibrated using the
% <matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'visionCameraCalibrator'); Camera Calibrator app>.
% The example uses a |viewSet| object to store and manage the data
% associated with each view, such as the camera pose and the image points,
% as well as matches between points from pairs of views. 
%
% The example uses the pairwise point matches to estimate the camera pose of
% the current view relative to the previous view. It then links the
% pairwise matches into longer point tracks spanning multiple views using
% the |findTracks| method of the |viewSet| object. These tracks then serve
% as inputs to multiview triangulation using the |triangulateMultiview|
% function and the refinement of camera poses and the 3-D scene points
% using the |bundleAdjustment| function.
%
% The example consists of two main parts: camera motion estimation and
% dense scene reconstruction. In the first part, the example estimates the
% camera pose for each view using a sparse set of points matched across the
% views. In the second part, the example iterates over the sequence of
% views again, using |vision.PointTracker| to track a dense set of points
% across the views, to compute a dense 3-D reconstruction of the scene.
%
% The camera motion estimation algorithm consists of the following steps:
%
% # For each pair of consecutive images, find a set of point
% correspondences. This example detects the interest points using the
% |detectSURFFeatures| function, extracts the feature descriptors using the
% |extractFeatures| functions, and finds the matches using the
% |matchFeatures| function. Alternatively, you can track the points across
% the views using |vision.PointTracker|.
% # Estimate the relative pose of the current view, which is the camera
% orientation and location relative to the previous view. The example uses
% a helper function |helperEstimateRelativePose|, which calls
% |estimateFundamentalMatrix| and |cameraPose|.
% # Transform the relative pose of the current view into the coordinate
% system of the first view of the sequence.
% # Store the current view attributes: the camera pose and the image
% points.
% # Store the inlier matches between the previous and the current view.
% # Find point tracks across all the views processed so far.
% # Use the |triangulateMultiview| function to compute the initial 3-D
% locations corresponding to the tracks.
% # Use the |bundleAdjustment| function to refine the camera poses and the
% 3-D points.
% Store the refined camera poses in the |viewSet| object.

%% Read the Input Image Sequence
% Read and display the image sequence.

% Use the |imageSet| object to get a list of all image file names in a
% directory.

%% Housekeeping
close all
clc
clear variables

plotEverything = true;
plotLoadMontage = false;
plotFirstSURF = false;
plotFirstValidPoints = false;
plotSubsequentMatches = false;
plotSubsequentSURF = false;

maxRatio = 0.5
%% Read the Input Image Sequence
% Read and display the image sequence.

% Use the |imageSet| object to get a list of all image file names in a
% directory.
imageDir = 'C:\Users\Brian\Desktop\cv\voltmeter4';
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
load(fullfile(imageDir, 'cameraParams.mat'));

%% Create a View Set Containing the First View
prevIm = undistortImage(images{1}, cameraParams);

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
    title('Top 100 SURF');
end

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
[prevFeatures, prevValidpoints] = extractFeatures(prevIm, prevPoints, ...
                                                  'Upright', false);
if plotEverything || plotFirstValidPoints
    figure; imshow(prevIm); hold on;
    plot(prevValidpoints,'showOrientation',true);
    title('ValidPoints Im1');
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

for i = 2:numel(images)
    fprintf('Processing Image %d\n', i);
    currIm = undistortImage(images{i}, cameraParams);
    
    % Detect, extract and match features. OG CODE
%     currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
%     currFeatures = extractFeatures(I, currPoints, 'Upright', true); 

    currPoints = detectSURFFeatures(currIm);
    [currFeatures, currValidpoints] = extractFeatures(currIm, currPoints, ...
                                                  'Upright', false);
                                              
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', maxRatio, 'Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    if plotEverything || plotSubsequentSURF
        figure;
        imshow(currIm);
        hold on
        plot(currValidpoints, 'showOrientation',true);
        title(sprintf('ValidPoints Im%d', i));
    end
    
    if plotEverything || plotSubsequentMatches
        prevImWidth = size(prevIm, 2);
        imPlot = [prevIm, currIm];
        
        loc1 = matchedPoints1.Location;
        loc2 = matchedPoints2.Location;
        
        figure;
        showMatchedFeatures(prevIm,currIm, loc1, loc2, 'montage');
%         imshow(imPlot);
%         hold on
%         for iLine = 1:size(loc1,1)
%             line([loc1(iLine,1), loc2(iLine,1) + prevImWidth], ...
%                  [loc1(iLine,2), loc2(iLine,2)]);
%         end
        title(sprintf('Image %d to %d', i-1, i));
        drawnow
    end
    
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
    
    fprintf('Number of inliers: %d\n', nnz(inlierIdx));
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
drawnow

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
    fprintf('Post Processing Image %d\n', i);
    
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
xlim([loc1(1)-3, loc1(1)+6]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+13]);
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
