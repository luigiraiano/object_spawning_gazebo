%% Luigi Raiano
clear all; close all; clc;
%%
% Compute the inertia matrix of a sphere
M = 0.25; % [kg]
R = 0.05; % [m]

ixx = (2/5)*M*R*R;
iyy = (2/5)*M*R*R;
izz = (2/5)*M*R*R;
ixy = 0;
iyx = 0;
ixz = 0;
izx = 0;
iyz = 0;
izy = 0;

I = [];
I = [ixx, ixy, ixz;
    iyx, iyy, iyz;
    izx, izy, izz];

disp('Interia Matrix')
disp(I)