%first download:
% http://vision.middlebury.edu/mview/data/data/dino.zip
% and extract to ../data
%
% This script is used to test multiview reconstruction given imeages with
% known camera calibration matrices K

%% Housekeeping
% close all
% clc
% clear variables

debug = true;

%% Set Paths (and User Parameters)
matchThresh = 1.5;

dinoDir = fullfile('..','data','dino');
dinoImExt = 'png';

% There is one line for each image. The format for each line is:
% "imgname.png k11 k12 k13 k21 k22 k23 k31 k32 k33
%              r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3"
% The projection matrix for that image is K*[R t]
cameraParameterPath = fullfile(dinoDir, 'dino_par.txt');
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
imPaths = getImPaths(dinoDir, dinoImExt);

%Shortening the dataset for testing purposes
imPaths = imPaths(1:10);
Ks = Ks(1:10);

SFM( imPaths, Ks )