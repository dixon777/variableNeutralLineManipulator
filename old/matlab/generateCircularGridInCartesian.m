function [xGrid, yGrid] = generateCircularGridInCartesian(minRadius, maxRadius, minAngle, maxAngle)

radiusDomain = linspace(minRadius, maxRadius, 50);
angleDomain = linspace(minAngle, maxAngle, 50);
[radiusGrid, angleGrid] = meshgrid(radiusDomain, angleDomain);
xGrid = radiusGrid .* cos(angleGrid);
yGrid = radiusGrid .* sin(angleGrid);
end
