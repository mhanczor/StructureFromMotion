function [ ptsConditioned, transform ] = preconditionPts( pts, avgDist )
%PRECONDITIONPTS Centers points around origin with avg dist sqrt(2)
%   Returns ptsConditioned, a matrix of size dxn Points, the centroid of the
%   points is the origin and the avgDist is the distance specified (default
%   sqrt(2).
%
%   transform is the (d+1)x(d+1) transformation matrix:
%           ptsConditioned = transform * pts
%
%
%   By: Brian Pugh

%% Check Inputs 
d = size(pts, 1);

if nargin < 2
    avgDist = sqrt(2);
end
if ~isscalar(avgDist)
    error('Optional Input avgDist must be a scalar');
end

%% Compute
centroid = mean(pts,2);
ptsCentered = bsxfun(@minus, pts, centroid);
distFromOrigin = sqrt(sum(ptsCentered .* ptsCentered));
avgDistFromOrigin = mean(distFromOrigin(:));
scale = avgDist / avgDistFromOrigin;
ptsConditioned = scale * ptsCentered;
transform = [scale*eye(d+1,d),[-scale*centroid; 1]];

end

