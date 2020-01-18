drawRingSurface(pi/2, 1, 3, generateTransformMatrix([1 0 0; 0 0 1; 0 1 0], [1,3,1]));

drawRingSurface(pi/2, 1, 3, eye(4));
function [x] = drawRingSurface(curvatureAngle, curvatureRadius, length, T)
    hold on;
    axis equal
    cylindricalRadius = curvatureRadius.*sin(curvatureAngle./2);
    angle_range = linspace(0,2*pi, 30);
    radius_range = linspace(0, cylindricalRadius, 30*cylindricalRadius);
    [angleG, radiusG] = meshgrid(angle_range, radius_range);
    
    xCurvatureG = radiusG.*cos(angleG);p
    yCurvatureG = radiusG.*sin(angleG);
    zCurvatureG = sqrt(curvatureRadius.^2-xCurvatureG.^2-yCurvatureG.^2+0.0000000000001);
    
    % bottom curvature
    [newX, newY, newZ] = transformMesh(xCurvatureG,yCurvatureG,-zCurvatureG, T*generateTransformMatrix([0,0,curvatureRadius]));
    surf(newX, newY, newZ);
    
    % top curvature
    [newX, newY, newZ] = transformMesh(xCurvatureG,yCurvatureG,zCurvatureG, T*generateTransformMatrix([0 0 length-curvatureRadius]));
    surf(newX, newY, newZ);
    
    % middle plane
    anglePlaneG = ones(100,1) .* angle_range;
    xPlaneG = cylindricalRadius.*cos(anglePlaneG);
    yPlaneG = cylindricalRadius.*sin(anglePlaneG);
    displacementFromBottom = curvatureRadius*(1-cos(curvatureAngle/2));
    zPlaneG =  ones(1,size(xPlaneG,2)).*(linspace(displacementFromBottom,length - displacementFromBottom, size(xPlaneG,1))');
    [newX, newY, newZ] = transformMesh(xPlaneG, yPlaneG, zPlaneG, T);
    surf(newX, newY, newZ);
    
    hold off;
end


function [newX, newY, newZ] = transformMesh(x, y, z, T)
    newX = reshape(x, 1,[]);
    newY = reshape(y, 1,[]);
    newZ = reshape(z, 1,[]);
    
    val = T*[newX;newY;newZ;ones(1,size(newX,2))];
    newX = reshape(val(1,:), size(x));
    newY = reshape(val(2,:), size(x));
    newZ = reshape(val(3,:), size(x));
end