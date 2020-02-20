radius = 2;

angleRange = linspace(0, 2*pi,100);
angleG = ones(100,1) .* angleRange;


x = radius.*cos(angleG);
y = radius.*sin(angleG);


surf(x,y, ones(1,size(x,2)).*(linspace(1,100, size(x,1))'));

