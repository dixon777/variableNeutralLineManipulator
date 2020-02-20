classdef VariableNeutralLineManipulator
    properties
        listParam;
    end
    methods
        function obj = VariableNeutralLineManipulator(listParam)
            obj.listParam = listParam;
        end
        
        function [max_length] = getMaxLength(obj)
            max_length = 0; % todo
        end
        
        function [max_orientation] = getMaxOrientation(obj)
            max_orientation = 0; % todo
        end
        
        function [extensions] = computeCableExtensions(obj, angles)
            % angles = Bending angles of each segment
            %   Row - Each segment starting from the most proximal part
            %   Col - If is2DoF for that segment is true, it should be to [pan_angle,
            %   tilt_angle], otherwise [bending_angle, 0]
            
            extensions = zeros(numel(obj.listParam), 4);
            
            % Change of angles for each joint of the respective segment
            inc_angles = zeros(numel(obj.listParam), 2);
            
            for segment_index=1:numel(obj.listParam)
                if(size(angles,1) < segment_index)
                    return
                end
                
                % Param of current segment
                param = obj.listParam{segment_index};
                
                % Update bending angle of each joint of the current segment
                if param.is2DoF
                    if numel(angles(segment_index, 1)) <= 1
                        angles(segment_index, 2) = 0;
                    end
                    inc_angles(segment_index,:) = [angles(segment_index, 1) / ceil(param.num_joints/2);angles(segment_index, 2)  / floor(param.num_joints/2)];
                else
                    inc_angles(segment_index,1) = angles(segment_index, 1)  / param.num_joints;
                end
                
                
                % Orientation of cables around the circular cross section
                % from the reference
                % If there are 2 DoF for this segment, then it has
                % 4 values, pi/2 radian apart, one located on the segment
                % starting_angle
                segment_cable_orientation_angles = param.getOrientationAngles();
                
                % Loop over all the previous segments since the extensions
                % of the current segment depends on all the more proximal
                % segments' bending configuration as well as its own
                for row=1:segment_index
                    
                    % Param of 1 previous segment
                    pre_param = obj.listParam{row};
                    
                    % Orientation of the bending with respect to the
                    % reference
                    % If there are 1 DoF for the previous segment, it
                    % has only 1 value, the starting angle. If there are 2
                    % DoF, it has an additional value pi/2 rad away from
                    % the starting angle
                    if pre_param.is2DoF
                        pre_orientation_angles = pre_param.starting_angle + [0;pi/2];
                    else
                        pre_orientation_angles = pre_param.starting_angle;
                    end
                    
                    % Evaluate the angle between the centre axis and the cables (from
                    % circular view to side view)
                    % Row - [pan direction, tilt direction] for 2 DoF, or
                    %   1 direction for 1 DoF
                    % Col - [pan direction]
                    beta = asin(param.curvature_radius.*sin(param.curvature_angle./2)./pre_param.curvature_radius.*(cos(segment_cable_orientation_angles - pre_orientation_angles )));
                    
                    % Evaluate change of length
                    extensions(row,:) = extensions(row,:) + sum(2.*pre_param.curvature_radius.*(cos(beta)-cos(beta-inc_angles(row,:)'./2)),1);
                end
            end
        end
        
        
        function [Ts] = getTFs(obj, totalRotatedAngles)
            Ts = {};
            prevT = eye(4);
            prevOrientation = 0;
            for i=1:numel(obj.listParam)
                param =  obj.listParam{i};
                Ts{i} = param.getTFs(totalRotatedAngles{i},prevOrientation, prevT);
                prevOrientation = param.starting_angle;
                prevT = Ts{end}(:,:,end);
            end
        end
        
        function draw(obj, totalRotatedAngles)
            Ts = obj.getTFs(totalRotatedAngles);
            prevT = eye(4);
            prevOrientation = 0;
            for i=1:numel(obj.listParam)
                param =  obj.listParam{i};
                if i<numel(obj.listParam)
                    nextParam = obj.listParam{i+1};
                else
                    nextParam = NaN;
                end
                param.draw(totalRotatedAngles{i},prevOrientation, prevT, nextParam);
                prevOrientation = param.starting_angle;
                prevT = Ts{i}(:,:,end);
            end
        end
    end
end



% 
% function T = getPanJointTransformMatrices(joint_angle, radius)
% cphi = cos(joint_angle/2);
% sphi = sin(joint_angle/2);
% 
% T_it_itd_pan = generateTransformMatrix([1 0 0; ...
%     0 cphi -sphi; ...
%     0 sphi cphi]);
% 
% T_itd_jbd = generateTransformMatrix([0,0, 2*radius*(1-cphi)]);
% 
% T_jbd_jb_pan = T_it_itd_pan;
% 
% 
% T = T_it_itd_pan*T_itd_jbd*T_jbd_jb_pan;
% end
% 
% function T = getTiltJointTransformMatrices(joint_angle, radius)
% cphi = cos(joint_angle/2);
% sphi = sin(joint_angle/2);
% 
% T_it_itd_tilt = generateTransformMatrix([cphi 0 sphi; ...
%     0  1 0; ...
%     -sphi 0 cphi]);
% 
% 
% T_itd_jbd = generateTransformMatrix([0,0, 2*radius*(1-cphi)]);
% 
% 
% T_jbd_jb_tilt = T_it_itd_tilt;
% 
% T = T_it_itd_tilt*T_itd_jbd*T_jbd_jb_tilt;
% end
% 



