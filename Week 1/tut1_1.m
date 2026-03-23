% =========================================================================
% Tutorial 1.1 - MATLAB Basics: Variables and Arithmetic
% =========================================================================
% PURPOSE:
%   Introduce MATLAB syntax by computing the volume of a geometric shape.
%   This builds the habit of defining named variables, using built-in
%   constants (pi), and writing readable arithmetic expressions — skills
%   used throughout all subsequent flight dynamics tutorials.
%
% CONCEPTS TAUGHT:
%   - Defining and assigning scalar variables
%   - Arithmetic operators and operator precedence
%   - Using the built-in constant pi
%   - Suppressing vs displaying output (semicolon usage)
%
% EXPECTED OUTPUT:
%   - Scalar value 'vol': cone volume in the same cubic units as r and h
% =========================================================================

% Cone parameters
r = 5;
h = 12;

% Compute the volume of the cone using variables r and h.
vol = (h*pi*r^2)/3;