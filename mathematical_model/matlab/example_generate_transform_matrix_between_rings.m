%% Definition
syms phi r L;

cphi = cos(phi/2);
sphi = sin(phi/2);

%% pan
T_it_itd_pan = generateTransformMatrix([1 0 0; ...
                0 cphi -sphi; ...
                0 sphi cphi]);
            
T_itd_jbd = generateTransformMatrix([0,0, 2*r*(1-cphi)]);

T_jbd_jb_pan = T_it_itd_pan;

T_jb_jt = generateTransformMatrix([0,0,L]);


T_it_jb_pan = T_it_itd_pan*T_itd_jbd*T_jbd_jb_pan;
T_it_jt_pan = T_it_jb_pan*T_jb_jt;

%% tilt
T_it_itd_tilt = generateTransformMatrix([cphi 0 sphi; ...
                0  1 0; ...
                -sphi 0 cphi]);

T_jbd_jb_tilt = T_it_itd_tilt;

% Utilize some t-matrices defined in 'pan'
T_it_jb_tilt = T_it_itd_tilt*T_itd_jbd*T_jbd_jb_tilt;
T_it_jt_tilt = T_it_jb_tilt*T_jb_jt;
    
