function [extensionFunc] = generte_VNLM_2DOF_CableExtensionFunction(curvature_radius, curvature_angle, num_joints, inRadian)
% Generate a function derive necessary displacement for each cable to
% achieve certain pan and tilt angles for a 2-DoF tendon-driven variable 
% neutral-line manipulator
    % Args:
    %   curvature_radius: Radius of curvature
    %   curvature_angle: Span angle between pair of cables
    %   inRadian: Whether all angles are in radian
    % Outputs: 
    %   extensionFunc - Function to derive 
    %       Args:
    %           1. Pan angle
    %           2. Tilt angle
    %       Outputs: 
    %           1. Left pan cable 
    %           2. Right pan cable 
    %           3. Left tilt cable
    %           4. Right tilt cable
    if inRadian
        f = @(x) x;
    else
        f = @(x) deg2rad(x);
    end
    half_curvature_angle = f(curvature_angle)/2;
    
    
    
    extensionFunc = @(pan_angle, tilt_angle) ...
    (2*num_joints*curvature_radius).*[(cos(half_curvature_angle)-cos(half_curvature_angle - f(pan_angle)/num_joints/2) + 1  - cos(f(tilt_angle)/num_joints/2));
    (cos(half_curvature_angle)-cos(half_curvature_angle + f(pan_angle)/num_joints/2) + 1  - cos(f(tilt_angle)/num_joints/2)) ;
    (cos(half_curvature_angle)-cos(half_curvature_angle - f(tilt_angle)/num_joints/2) + 1  - cos(f(pan_angle)/num_joints/2));
    (cos(half_curvature_angle)-cos(half_curvature_angle + f(tilt_angle)/num_joints/2) + 1  - cos(f(pan_angle)/num_joints/2))];

end