function m = generateTransformMatrix(rot, tran)
    % 
    % rot = Rotation matrix (3x3), tran = Translation vector (3x1 or 1x3)
    %
    % Usage:
    % generateTransformMatrix(rot, tran) => 
    % if size(tran) == [3,1] => [rot, tran;0 0 0 1] 
    % if size(tran) == [1,3] => [rot, tran';0 0 0 1] 
    %
    % generateTransformMatrix(rot) => [rot, [1;0;0];0 0 0 1]
    %
    % generateTransformMatrix(tran) => 
    % if size(tran) == [3,1] => [[1 0 0; 0 1 0; 0 0 1], tran;0 0 0 1] 
    % if size(tran) == [1,3] => [[1 0 0; 0 1 0; 0 0 1], tran';0 0 0 1] 
    if nargin == 1
        if all(size(rot) == [3,3])
            tran = zeros(3,1);
        elseif all(size(rot) == [3,1])
            tran = rot;
            rot = eye(3);
        elseif all(size(rot) == [1,3])
            tran = rot;
            rot = eye(3);
            tran = reshape(tran, 3,1);
        else
            error("Invalid size");
            
        end
    else
        if all(size(tran) == [1,3])
            tran = reshape(tran, 3, 1);
        end
    end
    m = [rot, tran; 0 0 0 1];
end