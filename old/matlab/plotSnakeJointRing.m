function plotSnakeJointRing(curvatureRadius, curvatureAngle, length, T, type)
[xBottomG, yBottomG, zBottomG, ...
xTopG, yTopG, zTopG, ...
xBodyG, yBodyG, zBodyG] = generateSnakeJointRing(curvatureRadius, curvatureAngle, length, type, T);

% [xBottomG, yBottomG, zBottomG] = transformCoors(xBottomG, yBottomG, zBottomG, T);
% [xTopG, yTopG, zTopG] = transformCoors(xTopG, yTopG, zTopG, T);
% [xBodyG, yBodyG, zBodyG]  = transformCoors(xBodyG, yBodyG, zBodyG, T);
hold on;
axis equal;
surf(xBottomG, yBottomG, zBottomG);
surf(xTopG, yTopG, zTopG);
surf(xBodyG, yBodyG, zBodyG);
xlabel("x");
ylabel("y");
zlabel("z");
hold off;
end

function [xBottomG, yBottomG, zBottomG, xTopG, yTopG, zTopG, xBodyG, yBodyG, zBodyG] = generateSnakeJointRing(curvatureRadius, curvatureAngle, length, feature, T)
[xBottomG, yBottomG, zBottomG] = generateSnakeJointRingCurvatureGrid(curvatureRadius, curvatureAngle, true);
if feature == "align"
    [xTopG, yTopG, zTopG] = transformCoors(xBottomG, yBottomG, -zBottomG, generateTransformMatrix([0 0 length]));
elseif feature == "alternate"
    [xTopG, yTopG, zTopG] = generateSnakeJointRingCurvatureGrid(curvatureRadius, curvatureAngle, false);
    [xTopG, yTopG, zTopG] = transformCoors(xTopG, yTopG, -zTopG, generateTransformMatrix([0 0 length]));
elseif feature == "end"
    [xTopG, yTopG, zTopG] = generateCircularFlatGrid(getSnakeJointCrossSectionalRadius(curvatureRadius, curvatureAngle));
     [xTopG, yTopG, zTopG] = transformCoors(xTopG, yTopG, zTopG, generateTransformMatrix([0 0 length]));
end

[xBottomG, yBottomG, zBottomG] = transformCoors(xBottomG, yBottomG, zBottomG, T);
[xTopG, yTopG, zTopG] = transformCoors(xTopG, yTopG, zTopG, T);
[xBodyG, yBodyG, zBodyG] = generateSnakeJointRingCylindricalBodyGrid(xBottomG(:,end), yBottomG(:,end), zBottomG(:,end), xTopG(:,end), yTopG(:,end), zTopG(:,end));


end










% Not gonna use it
function [xGrid, yGrid, zGrid] = generateSnakeJointRingCylindricalBodyGridOld(curvatureRadius, curvatureAngle, jointToJointLength)

circularRadius = curvatureRadius*sin(curvatureAngle/2);
angleDomain = linspace(0, 2*pi, 50);
radiusGrid = circularRadius + zeros(50, numel(angleDomain));
angleGrid = ones(50,1) * angleDomain;

xGrid = radiusGrid .* cos(angleGrid);
yGrid = radiusGrid .* sin(angleGrid);
displacementFromBottomCentre = curvatureRadius * (1-cos(curvatureAngle/2));
zGrid = linspace(displacementFromBottomCentre, jointToJointLength - displacementFromBottomCentre, size(angleGrid,1))' * ones(1,size(angleGrid,2));
end


