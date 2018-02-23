%% Housekeeping
close all;
clear variables;
clc;

%% Load in Data
% read in images
load(fullfile('..','data','intrinsics.mat'));
load(fullfile('..','data','some_corresp'));
im1 = imread(fullfile('..','data','im1.png'));
im2 = imread(fullfile('..','data','im2.png'));

nPoints = length(pts1);

%% Calculate the fundamental matrix
M = max([size(im1), size(im2)]);
[ F, bestInlierIdx ] = ransacF( pts1, pts2, M );

%% Remove Outliers
pts1 = pts1(bestInlierIdx, :);
pts2 = pts2(bestInlierIdx, :);

%% Calculate Essential MAtrix
[ E ] = essentialMatrix( F, K1, K2 );

%% Get camera projective matrices
% M = K * [R|t] where [R|t] is 3d rotation and translation.
% The camera matric M is sometimes denoted P in literature.
M1 = K1 * eye(3,4);
M2s = camera2(E);

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

%% Save Data
p1 = pts1;
p2 = pts2;
save('q2_5.mat', 'M2', 'p1', 'p2', 'P', 'error');