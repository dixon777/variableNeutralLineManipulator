% Applied force of the bending cable
x = linspace(-pi/4, pi/4,3000/pi);

c2 = cos(x/2);
s2 = sin(x/2);

figure(1);
plot(x, atan((1-c2)./s2)); % Orientation
figure(2);
plot(x, sqrt(s2.^2+(1-c2).^2)); % Magnitude
figure(3);
plot(x, -sqrt(s2.^2+(1-c2).^2).*cos(atan((1-c2)./s2))); % horizontal component
figure(4);
plot(x, sqrt(s2.^2+(1-c2).^2).*sin(atan((1-c2)./s2))); % vertical component

