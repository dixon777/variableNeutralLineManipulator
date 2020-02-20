function [xGrid, yGrid, zGrid] = generateSnakeJointRingCylindricalBodyGrid(x1, y1, z1, x2, y2, z2)
xGrid = [reshape(x1,1,[]); reshape(x2,1,[])];
yGrid = [reshape(y1,1,[]); reshape(y2,1,[])];
zGrid = [reshape(z1,1,[]); reshape(z2,1,[])];
% zGrid = linspace(displacementFromBottomCentre, jointToJointLength - displacementFromBottomCentre, size(angleGrid,1))' * ones(1,size(angleGrid,2));
end
