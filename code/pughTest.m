%% Generate a point prism
cubeDims = [27,7,67];
stepSize = [3,1,6];
x = []; y =[]; z = [];
for i = 0:5
    iMod = mod(i,3)+1;
    ranges = {1:stepSize(1):cubeDims(1); ...
              1:stepSize(2):cubeDims(2); ...
              1:stepSize(3):cubeDims(3)};
    ranges{iMod} = ones(1,length(ranges{iMod}));
    if i>=3
        ranges{iMod} = cubeDims(iMod) * ranges{iMod};
    end
    [xTmp, yTmp, zTmp] = meshgrid(ranges{1}, ranges{2}, ranges{3});
    x = [x,xTmp(:)'];
    y = [y,yTmp(:)'];
    z = [z,zTmp(:)'];
end

%% Plot 3D Cube
figure;
scatter(x,y,z);
hold on

%% Set Camera location
R1 = [
eye4 = eye(4);
rotPrimitive = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
rotX = @(theta) eye(
R1 = rotx(10) * roty(10) * rotz(10);
% cam1 = [


%% Generate views