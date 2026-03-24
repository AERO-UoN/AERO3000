function cy = rotatey(angle)
% Function Name: rotatey
%
% Function Description:
%   Performs rotations about the y-axis through a specified angle
%
% Inputs:
%   angle: Angle of rotation (in rad)
%
% Outputs:
%   cy: Rotation matrix
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Create matrix
cy = [ cos(angle),  0, -sin(angle);
    0,  1,           0;
    sin(angle),  0,  cos(angle)];
end