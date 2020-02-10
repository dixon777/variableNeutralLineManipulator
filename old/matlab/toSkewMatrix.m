
function [skewW] = toSkewMatrix(w)
w = reshape(w, 3,1);
skewW = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end