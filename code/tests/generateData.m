hemogenize = @(X) [X; ones(1,size(X,2))];
dehemogenize = @(X) bsxfun(@rdivide, X(1:end-1,:), X(end,:));

%% Generate camera matrices
K = [100    0   0; ...
       0   100  0; ...
       0    0   1];

% Camera 1: at origin, facing Z axis so camera XY is world XY
R1 = eye(3);
T1 = zeros(3,1);
P1 = K*[R1, T1];

% Camera 2: Shifted [10,0,0], Rotated inwards
th2 = -pi/24;
R2 = [cos(th2), 0, sin(th2); ...
             0, 1,        0; ...
     -sin(th2), 0, cos(th2)];
T2 = [10; 0; 0];
P2 = K*[R2, T2];

% Camera 3: shifted [20,5,5], Rotate even more inwards
th3 = -pi/12;
R3 = [cos(th3), 0, sin(th3); ...
             0, 1,        0; ...
     -sin(th3), 0, cos(th3)];
T3 = [20;5;5];
P3 = K*[R3, T3];

%% Generate points in 3d space, remember camera is at 0,0,0 facing 0,0,1
X = zeros(4,20);

X(3,:) = 20;
X(1,1:5) = 0:4;
X(2,6:10) = -2:2;
X(1,10:20) = 0:10;
X(2,10:20) = 3;
X(3,10:20) = 20:-.5:15;

% X = X + normrnd(0, .05, 4, 20);
X(4,:) = 1;
%% Project points into each camera
% Note that K has camera center at [0,0], so pixel values may be negative.
X1 = P1*X;
X1 = bsxfun(@rdivide, X1(1:2,:), X1(3,:));
X1 = X1 + normrnd(0, .01, 2, 20);

X2 = P2*X;
X2 = bsxfun(@rdivide, X2(1:2,:), X2(3,:));
X2 = X2 + normrnd(0, .01, 2, 20);

X3 = P3*X;
X3 = bsxfun(@rdivide, X3(1:2,:), X3(3,:));
X3 = X3 + normrnd(0, .01, 2, 20);


%% Normalized image coordinates
x1 = K\hemogenize(X1);

x2 = K\hemogenize(X2);

x3 = K\hemogenize(X3);

%% Visualize points
cla
hold on
scatter(X1(1,:), X1(2,:),'b')
scatter(X2(1,:), X2(2,:),'g')
scatter(X3(1,:), X3(2,:),'r')
hold off
