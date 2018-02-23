function [ F ] = eightpoint( pts1, pts2, M, refine )
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.1 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save F, M, pts1, pts2 to q2_1.mat

%     Write F and display the output of displayEpipolarF in your writeup

%% Parameter
% normalizeOptions = {'negOneToOne', 'zeroToOne'}
normalizeChoice = 2;

if nargin < 4 %default no refinement
    refine = false;
end

%% Reshape inputs
nPoints = length(pts1);

% In my original code, (x,y) coordinates were expressed as columns.
% now the points matrices are 3xN
pts1H = [pts1'; ones(1, nPoints)];
pts2H = [pts2'; ones(1, nPoints)];

%% Rescale
if normalizeChoice==1
    T = [2/M, 0, -1;
         0, 2/M, -1;
         0, 0, 1];
elseif normalizeChoice==2
    T = [1/M, 0, -0.5;
         0, 1/M, -0.5;
         0, 0, 1];
else
    error('invalid normalization choice');
end
pts1H = T * pts1H;
pts2H = T * pts2H;

%% Form A Matrix of Ax=0
% pts1^T * F * pts2 = 0
A = [pts2H(1,:)' .* pts1H(1,:)', ...
     pts2H(1,:)' .* pts1H(2,:)', ...
     pts2H(1,:)',               ...
     pts2H(2,:)' .* pts1H(1,:)', ...
     pts2H(2,:)' .* pts1H(2,:)', ...
     pts2H(2,:)',               ...
     pts1H(1,:)',               ...
     pts1H(2,:)',               ...
     ones(nPoints,1)];

%% Solve for the flatten E matrix using SVD
[~, ~, V] = svd(A, 'econ');
FNormalized = reshape(V(:,end),3,3)';

%% Enforce singularity
[U, S, V] = svd(FNormalized, 'econ');
S(3,3) = 0;
FNormalized = U * S * V';

%% Refine
if refine
    FNormalized = refineF(FNormalized, pts1, pts2);
end
%% Unscale
F = T' * FNormalized * T;

end
