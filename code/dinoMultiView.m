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

debug = true;

%% Set Paths (and User Parameters)
matchThresh = 1.5;
peakThresh = 3;

templeDir = fullfile('..','data','dino');
templeImExt = 'png';

% There is one line for each image. The format for each line is:
% "imgname.png k11 k12 k13 k21 k22 k23 k31 k32 k33
%              r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3"
% The projection matrix for that image is K*[R t]
cameraParameterPath = fullfile(templeDir, 'dino_par.txt');
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

% Well it's something
% scatter3(0,0,0,'rx');
% hold on
% for i = 1:10
%     plotCamera('Label', int2str(i), ...
%         'Orientation', Rs{i}, ...
%         'Location', -Rs{i}'*Ts{i}, ...
%         'Size', .01,...
%         'AxesVisible', true);
% end
% hold off
% axis equal

%% Find all image paths in the dino folder
imPaths = getImPaths(templeDir, templeImExt);
nImages = length(imPaths);


%% Process first image
i = 1;
prevIm = rgb2gray(imread(imPaths{i}));
pointsPrev = detectSURFFeatures(prevIm);
[featuresPrev,pointsPrev] = extractFeatures(prevIm,pointsPrev);

R1 = Rs{i};
T1 = -Rs{i}'*Ts{i};

vSet = viewSet;
% vSet = addView(vSet, i, 'Points', pointsPrev, 'Orientation', eye(3), ...
%   'Location', [0 0 0]);
vSet = addView(vSet, i, 'Points', pointsPrev, 'Orientation', R1, ...
      'Location', T1');
  
%% Add the rest
for i = 2:nImages % hardcoding this for reasons
  currentIm = rgb2gray(imread(imPaths{i}));
  points = detectSURFFeatures(currentIm);
  [features, points] = extractFeatures(currentIm, points); 
  
  
  R_pose = Rs{i};
  T_pose = -Rs{i}'*Ts{i};
%   R_rel = R_pose * R1';
%   T_rel = R1'*(T_pose - T1);
%   vSet = addView(vSet, i, 'Points', points, 'Orientation', R_rel, ...
%       'Location', T_rel');
  vSet = addView(vSet, i, 'Points', points, 'Orientation', R_pose, ...
      'Location', T_pose');

%   pairsIdx = matchFeatures(featuresPrev,features,'MatchThreshold',5);
  matches = matchFeatures(featuresPrev, features, ...
      'MatchThreshold',3, ...
      'Unique', true, ...
      'MaxRatio', 0.3);
%   
%   matchedPoints1 = pointsPrev.Location(matches(:,1),:);
%   matchedPoints2 = points.Location(matches(:,2),:);
%   
%   [~,inliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, ...
%       'Method', 'MSAC', 'DistanceThreshold', 1);
%   matches = matches(inliers,:);

  vSet = addConnection(vSet, i-1, i, 'Matches', matches);
  featuresPrev = features;
  
%   figure
%   showMatchedFeatures(prevIm, currentIm, pointsPrev.Location(matches(:,1),:), points.Location(matches(:,2),:));
  pointsPrev = points;
  prevIm = currentIm;
end

figure
%% Do the thing?
cameraParams = cameraParameters('IntrinsicMatrix', Ks{1}');
tracks = findTracks(vSet);
cameraPoses = poses(vSet);
[xyzPoints,errors] = triangulateMultiview(tracks, cameraPoses, cameraParams);

%% Visualize
pcshow(xyzPoints,'VerticalAxis','x', 'MarkerSize',30);
title('Before Bundle Adjustment');
% hold on
% for i = 1:size(cameraPoses,1);
%     plotCamera('Label', int2str(cameraPoses.ViewId(i)), ...
%         'Orientation', cameraPoses.Orientation{i}, ...
%         'Location', cameraPoses.Location{i}, 'AxesVisible', true, 'Size', 0.01);
% end
% hold off

[xyzPoints, cameraPoses] = bundleAdjustment(xyzPoints, tracks, cameraPoses, cameraParams);

figure
pcshow(xyzPoints,'VerticalAxis','x', 'MarkerSize',30);
title('After Bundle Adjustment');
% hold on
% for i = 1:size(cameraPoses,1);
%     plotCamera('Label', int2str(cameraPoses.ViewId(i)), ...
%         'Orientation', cameraPoses.Orientation{i}, ...
%         'Location', cameraPoses.Location{i}, 'AxesVisible', true, 'Size', 0.01);
% end
% hold off