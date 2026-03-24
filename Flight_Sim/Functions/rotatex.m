function cx = rotatex(angle)
%   Performs rotations about the x-axis through a specified angle
%
% Inputs:
%   angle: Angle of rotation (rad)
%
% Outputs:
%   cx: Rotation matrix
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Create matrix
cx = [ 1,           0,          0;
    0,  cos(angle), sin(angle);
    0, -sin(angle), cos(angle)];
end