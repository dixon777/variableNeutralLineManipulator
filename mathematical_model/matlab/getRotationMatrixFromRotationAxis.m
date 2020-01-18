function [R] = getRotationMatrixFromRotationAxis(w, theta)
skewW = toSkewMatrix(w/sum(sqrt(w)));
R = eye(3) + sin(theta) .* skewW + (1-cos(theta)) .* (skewW^2);
end