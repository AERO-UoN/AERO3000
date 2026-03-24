function cz = rotatez(angle)
%   Performs rotations about the z-axis through a specified angle
%
% Inputs:
%   angle: Angle of rotation (in rad)
%
% Outputs:
%   cz: Rotation matrix
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none
% Create matrix
cz = [ cos(angle),  sin(angle),  0;
    -sin(angle),  cos(angle),  0;
    0,           0,  1];

end