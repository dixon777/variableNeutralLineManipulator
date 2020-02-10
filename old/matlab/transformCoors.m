function [newX, newY, newZ] = transformCoors(x, y, z, T)
if all(T == eye(4), 'all')
    newX = x;
    newY = y;
    newZ = z;
    return;
end

originalSize = size(x);
x = reshape(x, 1, []);
y = reshape(y, 1, []);
z = reshape(z, 1, []);

newCoor = T*[x;y;z;ones(1, numel(x))];
newX = reshape(newCoor(1,:),originalSize);
newY = reshape(newCoor(2,:),originalSize);
newZ =reshape(newCoor(3,:),originalSize);
end
