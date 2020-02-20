function [xGrid, yGrid, zGrid] = generateCircularFlatGrid(radius)
[xGrid,yGrid]=generateCircularGridInCartesian(0, radius, 0, 2*pi);
zGrid = zeros(size(xGrid));
end