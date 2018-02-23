function [ R, t ] = estimateCameraPose( imgPoints, worldPoints, K )
%ESTIMATECAMERAPOSE Estimate camera pose based on known world <-> image
% correspondences
%
%   imgPoints - 2xN matrix of image points
%   worldPoints - 3xN matrix of world points
%   K - camera intrinsic matrix

    N = size(worldPoints,2);
    
    % Precondition
    [imgPoints, Timg] = preconditionPts(imgPoints(1:2,:));
    [worldPoints, Tworld] = preconditionPts(worldPoints(1:3,:));
    
    % Homogenize
    worldPoints = [worldPoints; ones(1,N)];
    
    % Create empty data matrix
    A = zeros(2*N, 12);
    
    xW = bsxfun(@times, worldPoints, imgPoints(1,:));
    yW = bsxfun(@times, worldPoints, imgPoints(2,:));

    A(1:N,1:4) = worldPoints';
    A(1:N,9:12) = -xW';
    A(N+1:end,5:8) = worldPoints';
    A(N+1:end,9:12) = -yW';
   
    % Compute nullspace
    [~,~,v] = svd(A'*A,0);
    p = v(:,end);
    
    M = reshape(p,4,3)';
    
    % Undo preconditioning
    M = Timg\M*Tworld;
    
    % Extract extrinsics
    P = K\M;
    R = P(:,1:3);
    t = P(:,4);
    
    % determine scale factor
    d = det(R);
    alpha = 1 / nthroot(d, 3);
    
    R = alpha*R;
    t = alpha*t;
    
    % enforce no reflection
    [u,~,v] = svd(R);
    s = eye(3);
    s(3,3) = sign(det(u*v'));
    R = u*s*v';
end

