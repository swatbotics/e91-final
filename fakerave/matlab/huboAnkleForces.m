%% huboAnkleForces.m
% The "huboAnkleForces" script does a sanity check on the forces on the
% ankles of Hubo if its CoM is accelerating in the X-Y plane.
%
% NOTES:
%
% NECESSARY FILES AND/OR PACKAGES:
%
% SEE ALSO:
%
% AUTHOR:
%    Rowland O'Flaherty (www.rowlandoflaherty.com)
%
% VERSION: 
%   Created 14-MAR-2013
%-------------------------------------------------------------------------------

%% Clean up
close all; clc; clear all

%% Create variables
% Coordinate system is at right foot under ankle (x forward, y towards left
% foot, z up)
syms x % CoM x position from right foot
syms c % CoM y position from right foot
syms h % CoM z position from right foot
syms d % Left foot x position from right foot
syms m xdd ydd g % Mass, x acceleration, y acceleration, gravity
syms fr fl frx fry frz flx fly flz % Right Left foot force vectors and components
syms tr tl % Right Left torque vectors

%% Create force and torque vectors
fr = [frx;fry;frz]
fl = [flx;fly;flz]

tr = cross([-x;-c;-h],fr)
tl = cross([-x;d-c;-h],fl)

%% Solve for left forces interms of right forces with equation sum of forces equal zeros
sol = solve(fr+fl-[m*xdd;m*ydd;m*g],flx,fly,flz);
sol.flx
sol.fly
sol.flz

%% Subsitute solution into left torque
tl_s = subs(tl,{flx,fly,flz},{sol.flx,sol.fly,sol.flz})

%% Solve for right x and z forces in terms of known variables
frz_s = solve(tl_s(1) + tr(1),frz)
frx_s = solve(tl_s(3) + tr(3),frx)

%% Simplify solution of y torque equation
simplify(tl_s(2) + tr(2))

%% Subsitute solution back into left force equations
flx_s = subs(sol.flx,frx,frx_s)
flz_s = subs(sol.flz,frz,frz_s)


%% Print results
fprintf('=================================\n');
fprintf('Right X and Z forces:\n')
fr_s = [frx_s;nan;frz_s]
fprintf('Left X and Z forces:\n')
fl_s = [flx_s;nan;flz_s]