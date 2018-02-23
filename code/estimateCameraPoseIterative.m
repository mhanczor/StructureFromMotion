function [ R, t ] = estimateCameraPoseIterative( imgPoints, worldPoints, K , Pestimate)
%ESTIMATECAMERAPOSEITERATIVE Iteratively estimate camera pose based on known world <-> image
% correspondences. Uses Levenberg–Marquardt.
%
%   imgPoints - 2xN matrix of image points
%   worldPoints - 3xN matrix of world points
%   K - camera intrinsic matrix
%   Pestimate - (optional) initial value for P. Defaults to K*[eye, 0].

    %% Params
    if ~exist('Pestimate', 'var')
        Pestimate = K*[eye(3), zeros(3,1)];
    end
    
    nPoints = size(worldPoints,2);
    nParams = 12;
    maxIter = 250;
    
    % Precondition
    [imgPoints, Timg] = preconditionPts(imgPoints(1:2,:));
%     [worldPoints, Tworld] = preconditionPts(worldPoints(1:3,:));
    worldPoints = worldPoints(1:3,:);
    Tworld = eye(4);
    
    % Homogenize
    worldPoints = [worldPoints; ones(1,nPoints)];
    
    P = Timg*Pestimate/Tworld;
    
    lambda = 1/10;
    error = Inf;
    updateJ = 1;
    grad = Inf*ones(12,1);
    iter = 1;
    while max(grad) > eps && iter < maxIter
        % Update Jacobian if necessary
        if updateJ
            J = zeros(2*nPoints, nParams);

            d1 = P(1,:)*worldPoints;
            d2 = P(2,:)*worldPoints;
            d3 = P(3,:)*worldPoints;
            oneOverD3 = 1./d3;

            J(1:nPoints, 1:4) = bsxfun(@times, worldPoints, oneOverD3)';
            J(nPoints+1:end, 5:8) = bsxfun(@times, worldPoints, oneOverD3)';

            J(1:nPoints,9:12)     = -bsxfun(@times, worldPoints, d1 .* oneOverD3.^2)';
            J(nPoints+1:end,9:12) = -bsxfun(@times, worldPoints, d2 .* oneOverD3.^2)';
            
            % Calculate residual
            ex = imgPoints(1,:) - d1 .* oneOverD3;
            ey = imgPoints(2,:) - d2 .* oneOverD3;
            r = [ex, ey]';
            grad = J'*r;
            
            % Approximate Hessian
            H = J'*J;
        end
        
        % Add damping
        H_dampened = H + lambda*diag(diag(H));
                
        % Calculate update
        dp = H_dampened\grad;
        
        dP = reshape(dp, 4, 3)';
        P_candidate = P + dP;
        
        % Evaluate step
        projected = P_candidate * worldPoints;
        projected = bsxfun(@rdivide, projected(1:2,:), projected(3,:));
        diff = imgPoints - projected;
        error_candidate = sum(diff(:).^2);
        
        if error_candidate < error
            lambda = lambda / 10;
            P = P_candidate;
            error = error_candidate;
            updateJ = 1;
        else
            updateJ = 0;
            lambda = lambda*10;
        end
        
        iter = iter + 1;
    end
    fprintf('Converged in %i iterations\n', iter);
    
    % Undo preconditioning
    M = Timg\P*Tworld;
        
    % Extract extrinsics
    P = K\M;
    R = P(:,1:3);
    t = P(:,4);
    
    % Determine scale factor
    d = det(R);
    alpha = 1 / nthroot(d, 3);
    
    R = alpha*R;
    t = alpha*t;
    
    % Enforce no reflection
    [U,~,V] = svd(R);
    S = eye(3);
    S(3,3) = sign(det(U*V'));
    R = U*S*V';
end

