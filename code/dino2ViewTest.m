%first download:
% http://vision.middlebury.edu/mview/data/data/dino.zip
% and extract to ../data
%
% This script is used to test VLFEAT feature detectors and feature
% matching, and then to use these point correspondances to construct a
% sparse 3D pointcloud from two images.

%% Housekeeping
close all
clc
clear variables

debug = true;

%% Set Paths (and User Parameters)
matchThresh = 1.5;

dinoDir = fullfile('..','data','dino');
im1Number = 1;
im2Number = 3;
im1Filename = ['dino' num2str(im1Number, '%04d') '.png'];
im2Filename = ['dino' num2str(im2Number, '%04d') '.png'];
im1FullPath = fullfile(dinoDir, im1Filename);
im2FullPath = fullfile(dinoDir, im2Filename);

% There is one line for each image. The format for each line is:
% "imgname.png k11 k12 k13 k21 k22 k23 k31 k32 k33
%              r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3"
% The projection matrix for that image is K*[R t]
cameraParameterPath = fullfile(dinoDir, 'dino_par.txt');
cameraParameterDelimiter = ' ';
cameraParameterNHeadLines = 1;

%% Load images and camera intrinsics
im1Color = imread(im1FullPath);
im2Color = imread(im2FullPath);
im1Gray = single(rgb2gray(im1Color));
im2Gray = single(rgb2gray(im2Color));

% cameraParameterReadIn is a struct with two fields:
%   data
%   textdata
cameraParameterReadIn = importdata(cameraParameterPath , ...
                                   cameraParameterDelimiter, ...
                                   cameraParameterNHeadLines);
cameraParameterReadIn.textdata(1) = []; %first value is garbage

%% Parse Camera Intrinsics
nImages = size(cameraParameterReadIn.textdata, 1);
for i = 1:nImages
    rowData = cameraParameterReadIn.data(i,:);
    if strcmp(cameraParameterReadIn.textdata{i}, im1Filename)
        K1 = vec2mat(rowData(1:9), 3);
        R1 = vec2mat(rowData(10:18), 3);
        T1 = vec2mat(rowData(19:end), 1);
    elseif strcmp(cameraParameterReadIn.textdata{i}, im2Filename)
        K2 = vec2mat(rowData(1:9), 3);
        R2 = vec2mat(rowData(10:18), 3);
        T2 = vec2mat(rowData(19:end), 1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BELOW SHOULD BE IN SOME FUNCTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup the VLFeat Toolbox
VLFEATROOT = fullfile('..','vlfeat-0.9.20');
run(fullfile(VLFEATROOT, 'toolbox', 'vl_setup'));
fprintf('VLFeat Version: %s.\n', vl_version);

%% Perform SIFT
[ features, descriptors, matches, scores ] = ...
            SIFT( {im1FullPath, im2FullPath}, matchThresh );
        
%% Debug Test Plot
if debug
    figure;imagesc(im1Color);
    hold on;
    vl_plotframe(features{1});
    figure;imagesc(im2Color);
    hold on;
    vl_plotframe(features{2});
end

%% Estimate F using RANSAC
pts1 = features{1}(1:2, matches{1,2}(1,:))'; % each row is an (x,y) coordinate
pts2 = features{2}(1:2, matches{1,2}(2,:))';
M = max([size(im1Gray), size(im2Gray)]); %max image dimension
[ F, bestInlierIdx ] = ransacF( pts1, pts2, M );

pts1 = pts1(bestInlierIdx, :);
pts2 = pts2(bestInlierIdx, :);

%% Debug, Check F by epipolar match gui
if debug
    figure;
    displayEpipolarF(im1Color, im2Color, F);
end

%% Calculate E Essential Matrix From Known Camera Intrinsics
[ E ] = essentialMatrix( F, K1, K2 );

%% Get camera projective matrices
% M = K * [R|t] where [R|t] is 3d rotation and translation.
% The camera matric M is sometimes denoted P in literature.
M1 = K1 * eye(3,4);
M2s = camera2(E, eye(3), zeros(3,1));

%% Select the correct M2
for i = 1:4
    M2 = K2*M2s(:,:,i);
    [ P, error ] = triangulate( M1, pts1, M2, pts2 );
    
    %% Check if points are in front of camera and break early if possible
    % check if points are in front of camera
    if any(P(:,3) < 0) == false 
        break;
    elseif i==4
        error('No valid M2 !?!');
    end
end

%% Triangulate
[ P, error ] = triangulate( M1, pts1, ...
                            M2, pts2);

%% 3D Plot
figure;
scatter3(P(:,1), P(:,2), P(:,3), '.');
camproj('perspective')
