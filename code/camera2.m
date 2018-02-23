function [M2s] = camera2(E, R1, T1)
%CAMERA2 Compute possible extrinsics of camera 2 given essential matrix
%between camera 1 and 2, and extrinsics of camera 1.
%
% E  - Essential matrix s.t. x1' E x2 = 0
% R1 - Rotation of camera 1
% T1 - Translation of camera 1

[U,S,V] = svd(E);
m = (S(1,1)+S(2,2))/2;
E = U*[m,0,0;0,m,0;0,0,0]*V';
[U,S,V] = svd(E);
W = [0,-1,0;1,0,0;0,0,1];

% Make sure we return rotation matrices with det(R) == 1
if (det(U*W*V')<0)
    W = -W;
end

M2s = zeros(3,4,4);
M2s(:,:,1) = [U*W*V'*R1,U(:,3)./max(abs(U(:,3))) + T1];
M2s(:,:,2) = [U*W*V'*R1,-U(:,3)./max(abs(U(:,3))) + T1];
M2s(:,:,3) = [U*W'*V'*R1,U(:,3)./max(abs(U(:,3))) + T1];
M2s(:,:,4) = [U*W'*V'*R1,-U(:,3)./max(abs(U(:,3))) + T1];
end
