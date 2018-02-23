function [ F ] = sevenpoint( pts1, pts2, M )
% sevenpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.2 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save recovered F (either 1 or 3 in cell), M, pts1, pts2 to q2_2.mat

%     Write recovered F and display the output of displayEpipolarF in your writeup

%% Parameter
% normalizeOptions = {'negOneToOne', 'zeroToOne'}
normalizeChoice = 2;
imagTol = 0.001;

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
[U, S, V] = svd(A);
FN1 = reshape(V(:,end-1),3,3)';
FN2 = reshape(V(:,end),3,3)';

%% Solve for lambda
lambda = -eig(FN2\FN1);
lambda(~imag(lambda)==0) = []; %remove complex soln

nSoln = length(lambda);
F = cell(nSoln, 1);

%% Rescale and package output
for i = 1:nSoln
    [U, S, V] = svd(T' * (FN1 + lambda(i)*FN2) * T);
    S(3,3) = 0;
    F{i} = U * S * V';
end

end

