syms th Fr N %Tl Tr
L = 3;
r = 4;
alpha = pi/2;
Tl = 1;
Tr = 1;

c2 = cos(th/2);
s2 = sin(th/2);
ca2 = cos(alpha/2);
sa2 = sin(alpha/2);

ff = @(x) ((heaviside(x)-0.5).*2);

gm = atan((1-c2)./s2);
Cnl_F = ff(th).* Tl.*sqrt(s2.^2+(1-c2).^2).*[-cos(gm); sin(gm)];
Cnr_F = ff(th).* Tr.*sqrt(s2.^2+(1-c2).^2).*[-cos(gm); sin(gm)];
Tl_F = [0; -Tl];
Tr_F = [0; -Tr];
N_F = N.*[s2;c2];
Fr_F = Fr.*[c2;-s2];



Cn1_r = r.*[sa2; ca2];
Cnr_r = r.*[-sa2; ca2];
Tl_r = [r.*sa2; -(L-r)];
Tr_r = [-r.*sa2; -(L-r)];
N_r = r.*[s2; c2];
Fr_r = r.*[s2; c2];

F_eq_th_only = Cnl_F + Cnr_F + Tl_F + Tr_F;
F_eq_N_Fr = [-(F_eq_th_only(1).*s2+F_eq_th_only(2).*c2); -(F_eq_th_only(1).*c2 - F_eq_th_only(2).*s2)];


M_eq = cross2d(Cnl_F, Cn1_r) + ...
    cross2d(Cnr_F, Cnr_r) + ...
    cross2d(Tl_F, Tl_r) + ...
    cross2d(Tr_F, Tr_r) + ...
    cross2d(N_F, N_r) + ...
    cross2d(Fr_F, Fr_r);


th_v_min = -(alpha/2);
th_v_max = alpha/2;

% while true
%     th_v = (th_v_min + th_v_max)/2+eps;
%     res = subs(F_eq_N_Fr, th, th_v);
%     N_v = res(1);
%     Fr_v = res(2);
%     res = eval(subs(M_eq, [N, Fr, th], [N_v, Fr_v, th_v]));
%     if abs(res) < 0.0001
%         fprintf("Joint bending angle is at: %d degree\n", rad2deg(  th_v));
%         break
%     elseif res > 0
%         th_v_min = th_v;
%     else
%         th_v_max = th_v;
%     end
% end


th_range = -alpha/2:0.1:alpha/2;

N_vs = subs(F_eq_N_Fr(1), th, th_range);
Fr_vs = subs(F_eq_N_Fr(2), th, th_range) ;
convergence_vs =  eval(subs(M_eq, {N, Fr, th}, {N_vs, Fr_vs, th_range}));

figure(1);
plot(th_range,convergence_vs);
title('Convergence result');
xlabel('Joint bending angle (rad)');
displayAtOrigin();

figure(2);
plot(th_range, N_vs);
title('Normal force at various joint bending angle');
displayAtOrigin();

figure(3);
plot(th_range, Fr_vs);
title('Frictional force at various joint bending angle');
displayAtOrigin();

figure(4);
plot(th_range, subs(Cnl_F, th, th_range));
title('Applied force on left cable at bending at various joint bending angle');
displayAtOrigin();

figure(5);
plot(th_range, subs(Cnr_F, th, th_range));
title('Applied force on right cable at bending at various joint bending angle');
displayAtOrigin();



function res = cross2d(a,b)
    res = a(1)*b(2) - a(2)*b(1);
end

function displayAtOrigin()
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
end