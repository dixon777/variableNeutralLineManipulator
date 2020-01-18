

classdef VariableNeutralLineManipulatorParam
    % Param definition for a segment of an arm (Used in conjunction with
    % VariableNeutralLineManipulator)
    %
    % Required args:
    %   is2DoF - whether it is a 2DoF segment
    %
    %   starting_angle - Angle (in radian), from the reference, of the initial cable
    % for actuation of this segment (For example, if it is given as pi/3, and is2DoF
    % is true, the cables are located at pi/3, (pi/3+pi/2), (pi/3+pi),
    % (pi/3+3*pi/4) respectively)
    %
    %   curvature_radius - Radius of the curvature of the contacting
    % surface
    %
    %   curvature_angle - Span angle of curvature of the contacting
    % surface
    %
    %   num joints - Number joints along the segment
    %
    %   component length - Length of each component of this segment
    %
    %   last component length (optional) - Length of most distal component of
    % this segment. (Should be defined only if that component has a
    % different length than the rest of the component of the same segment)
    
    properties
        is2DoF % bool
        starting_angle % double
        curvature_angle % double
        curvature_radius % double
        num_joints % int
        component_length % double
        last_component_length % double [optional]
        
    end
    methods
        function obj = VariableNeutralLineManipulatorParam(is2DoF, starting_angle, curvature_radius, curvature_angle, num_joints, component_length, last_component_length)
            if nargin < 6
                error("Must have 6 args, but you only input %d args", nargin);
            elseif (is2DoF && num_joints < 2) || (~is2DoF && num_joints < 1)
                if is2DoF 
                    num_args = 2;
                else
                    num_args = 1;
                end
                error("num_joints must be at least %d", num_args);
            end
            obj.is2DoF = is2DoF;
            obj.starting_angle = starting_angle;
            obj.curvature_angle = curvature_angle;
            obj.curvature_radius = curvature_radius;
            obj.num_joints = num_joints;
            obj.component_length = component_length;
            if nargin <= 6
                obj.last_component_length = component_length;
            else
                obj.last_component_length = last_component_length;  
            end
        end
        
        function angles = getOrientationAngles(obj)
            if obj.is2DoF 
                angles = [0, pi, pi/2, 3*pi/2];
            else
                angles = [0, pi];
            end
            angles  = angles + obj.starting_angle;
        end
        
        function Ts = getTFs(obj, totalRotatedAngle, prevOrientation, prevT)
            Ts = zeros(4,4,obj.num_joints);
            if nargin==3
                 T = generateTransformMatrix(getRotationMatrixFromRotationAxis([0,0,1], obj.starting_angle - prevOrientation));
            else
                 T = prevT*generateTransformMatrix(getRotationMatrixFromRotationAxis([0,0,1], obj.starting_angle - prevOrientation));
            end
           
            Ts(:,:,1) = T;
            TsIndex = 2;
            if obj.is2DoF
                jointPanAngle = obj.getJointAngle(totalRotatedAngle(1), true);
                jointTiltAngle = obj.getJointAngle(totalRotatedAngle(2), false);
                jointTFPan = obj.getJointTF(jointPanAngle, true);
                jointTFTilt = obj.getJointTF(jointTiltAngle, false);
                
                for i=1:obj.num_joints
                    if mod(i,2) == 1
                        T = T * jointTFPan;
                    else
                        T = T* jointTFTilt;
                    end
                        
                    if i==obj.num_joints
                        T = T*generateTransformMatrix([0 0 obj.last_component_length]);
                    else
                        T = T*generateTransformMatrix([0 0 obj.component_length]);
                    end
                    Ts(:,:,TsIndex) = T;
                    TsIndex = TsIndex+1;
                end

            else
                jointPanAngle = obj.getJointAngle(totalRotatedAngle(1), true);
                jointTFPan = obj.getJointTF(jointPanAngle, true);
                 for i=1:obj.num_joints
                    T = T * jointTFPan;
                    if i==obj.num_joints
                        T = T*generateTransformMatrix([0 0 obj.component_length]);
                    else
                        T = T*generateTransformMatrix([0 0 obj.last_component_length]);
                    end
                    Ts(:,:,TsIndex) = T;
                    TsIndex = TsIndex+1;
                end
            end
            
            
        end
        
        function draw(obj, totalRotatedAngle, prevOrientation, prevT, nextParam)
            Ts = obj.getTFs( totalRotatedAngle, prevOrientation, prevT);
            [xGrid, yGrid, zGrid] = obj.getInitTopCurvatureGrid(prevOrientation, prevT);
            drawGrids(xGrid, yGrid, zGrid);
            
            jointPanAngle = obj.getJointAngle(totalRotatedAngle(1), true);
            jointTiltAngle = obj.getJointAngle(totalRotatedAngle(2), false);
            jointTFPan = obj.getJointTF(jointPanAngle, true);
            jointTFTilt = obj.getJointTF(jointTiltAngle, false);
            
            for i=1:obj.num_joints
                if obj.is2DoF
                    if mod(i,2) == 1
                        extraT = jointTFPan*eye(4);
                    else
                        extraT = jointTFTilt*generateTransformMatrix(getRotationMatrixFromRotationAxis([0,0,1], pi/2));
                    end
                    if i==obj.num_joints
                        if isnan(nextParam)
                            type = "end";
                            length = obj.last_component_length;
                        else
                            [xBottomG, yBottomG, zBottomG] = generateSnakeJointRingCurvatureGrid(curvatureRadius, curvatureAngle);
                            [xBottomG, yBottomG, zBottomG] = transformCoors(xBottomG, yBottomG, zBottomG, Ts(:,:,i)*extraT);
                            [xTopG, yTopG, zTopG] = nextParam.getInitTopCurvatureGrid(obj.starting_angle, Ts(:,:,i)*extraT);
                            [xTopG, yTopG, zTopG] = transformCoors(xTopG, yTopG, zTopG, generateTransformMatrix([0 0 obj.last_component_length]));
                            [xBodyG, yBodyG, zBodyG] = generateSnakeJointRingCylindricalBodyGrid(xBottomG, yBottomG, zBottomG,xTopG, yTopG, zTopG);
                            drawGrids(xBottomG, yBottomG, zBottomG);
                            drawGrids(xBodyG, yBodyG, zBodyG);
                            continue;
                        end
                    else
                        type = "alternate";
                        length = obj.component_length;
                    end
                    plotSnakeJointRing(obj.curvature_radius, obj.curvature_angle, length, Ts(:,:,i)*extraT, type);
                else
                    if i==obj.num_joints
                        type = "end";
                    else
                        type = "align";
                    end
                    plotSnakeJointRing(obj.curvature_radius, obj.curvature_angle, obj.component_length, Ts(:,:,i), type);
                end
                
            end
        end
        
        function [xGrid, yGrid, zGrid] = getInitTopCurvatureGrid(obj, prevOrientation, prevT)
            [xGrid, yGrid, zGrid] = generateSnakeJointRingCurvatureGrid(obj.curvature_radius, obj.curvature_angle, true);
            [xGrid, yGrid, zGrid] = transformCoors(xGrid, yGrid, -zGrid, prevT*generateTransformMatrix(getRotationMatrixFromRotationAxis([0,0,1], obj.starting_angle - prevOrientation)));
        end
        
        function jointRotatedAngle = getJointAngle(obj, totalRotatedAngle, isPan)
            if obj.is2DoF
                if isPan
                    jointRotatedAngle = totalRotatedAngle / ceil(obj.num_joints/2);
                else
                     jointRotatedAngle = totalRotatedAngle / floor(obj.num_joints/2);
                end
            else
                jointRotatedAngle = totalRotatedAngle / obj.num_joints;
            end
            
        end
        
        
        function T = getJointTF(obj, jointRotatedAngle, isPan)
            % Return tf of 1 joint
            if ~obj.is2DoF || isPan
                T_it_itd = generateTransformMatrix(getRotationMatrixFromRotationAxis([1,0,0], jointRotatedAngle/2));
            else
                T_it_itd = generateTransformMatrix(getRotationMatrixFromRotationAxis([0,1,0], jointRotatedAngle/2));
            end
            T_itd_jbd = generateTransformMatrix([0,0, 2*obj.curvature_radius*(1-cos(jointRotatedAngle/2))]);
            
            T = T_it_itd*T_itd_jbd*T_it_itd;
        end
    end
end



function drawGrids(xGrid, yGrid, zGrid)
hold on;
axis equal;
surf(xGrid, yGrid, zGrid);
xlabel("x");
ylabel("y");
zlabel("z");
hold off;
end