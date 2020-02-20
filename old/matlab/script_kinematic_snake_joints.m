% Function
%% 1 DoF
r = 1;
num_joints = 2;
curvature_angle = deg2rad(45);
func = generte_VNLM_1DOF_CableExtensionFunction(r, curvature_angle, num_joints, true);

bending_angle = deg2rad(2);
func(bending_angle)



%% 2 DoF
r = 1;
num_joints = 2;
curvature_angle = deg2rad(45);
func = generte_VNLM_2DOF_CableExtensionFunction(r, curvature_angle, num_joints,
true);

tilt_angle = deg2rad(2);
pan_angle = deg2rad(2);
func(pan_angle,tilt_angle);


% Class
%% 2 DoF
a = VariableNeutralLineManipulatorParam(true, 0, 2, pi/2, 4, 3);
computer = VariableNeutralLineManipulator({a});
% computer.computeCableExtensions([pi/2,]);

computer.draw({[pi/3,pi/4]})



