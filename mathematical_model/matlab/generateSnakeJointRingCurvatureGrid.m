function [xCurvatureGrid, yCurvatureGrid, zCurvatureGrid] = generateSnakeJointRingCurvatureGrid(curvatureRadius, curvatureAngle, isPan)

[xCurvatureGrid,yCurvatureGrid] = generateCircularGridInCartesian(0, getSnakeJointCrossSectionalRadius(curvatureRadius,curvatureAngle), 0, 2*pi);

if isPan
    zTemp = curvatureRadius.^2 - yCurvatureGrid.^2;
else
    zTemp = curvatureRadius.^2 - xCurvatureGrid.^2;
end
zTemp(abs(zTemp) < eps) = 0;
zCurvatureGrid = sqrt(zTemp);

[xCurvatureGrid, yCurvatureGrid, zCurvatureGrid] = transformCoors(xCurvatureGrid, yCurvatureGrid, -zCurvatureGrid, generateTransformMatrix([0 0 curvatureRadius]));
end
% function [xCurvatureGrid, yCurvatureGrid, zCurvatureGrid] = generateSnakeJointRingCurvatureGrid(curvatureRadius, curvatureAngle, isPan)
% 
% [xCurvatureGrid,yCurvatureGrid] = generateCircularGridInCartesian(0, getSnakeJointCrossSectionalRadius(curvatureRadius,curvatureAngle), 0, 2*pi);
% 
% if isPan
% zTemp = curvatureRadius.^2 - yCurvatureGrid.^2;
% else
%     zTemp = curvatureRadius.^2 - xCurvatureGrid.^2;
% end
% zTemp(abs(zTemp) < eps) = 0;
% zCurvatureGrid = sqrt(zTemp);
% 
% [xCurvatureGrid, yCurvatureGrid, zCurvatureGrid] = transformCoors(xCurvatureGrid, yCurvatureGrid, -zCurvatureGrid, generateTransformMatrix([0 0 curvatureRadius]));
% end
