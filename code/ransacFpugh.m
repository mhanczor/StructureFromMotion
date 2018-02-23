function [ F, bestInlierIdx ] = ransacFpugh( pts1, pts2, M )
% ransacF:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.X - Extra Credit:
%     Implement RANSAC
%     Generate a matrix F from some '../data/some_corresp_noisy.mat'
%          - using sevenpoint
%          - using ransac

%     In your writeup, describe your algorith, how you determined which
%     points are inliers, and any other optimizations you made

%% Tunable Parameters
inlierRadius = 2; %pixel radius for inlier classification
nItr = 1000; %Number of RANSAC to perform

%% Paramter Calculation
nPoints = length(pts1);
pts1H = [pts1'; ones(1, nPoints)];
pts2H = [pts2'; ones(1, nPoints)];

%% RANSAC ITERATION using 7pt algo
bestF = [];
bestInlierIdx = [];
nBestInliers = 0;
for i = 1:nItr
    ptsIdx = randperm(nPoints,7);
    pts17 = pts1(ptsIdx,:);
    pts27 = pts2(ptsIdx,:);
    [ Fs ] = sevenpoint( pts17, pts27, M );
    for j = 1: length(Fs)
        F = Fs{j}';
        Fp = F * pts1H;
        distanceNumerator = abs(sum(pts2H .* Fp, 1));
        distanceDenominator = sqrt(Fp(1,:).^2 + Fp(2,:).^2);
        dist = distanceNumerator ./ distanceDenominator;
        inlierIdx = dist < inlierRadius;
        nInlier = sum(inlierIdx);
        if nInlier > nBestInliers
            bestF = F;
            bestInlierIdx = inlierIdx;
            nBestInliers = nInlier;
        end
    end
end

%% Recalculate F with all inliers using the 8pt algo
[ F ] = eightpoint( pts1(bestInlierIdx, :), pts2(bestInlierIdx, :), M );
end

