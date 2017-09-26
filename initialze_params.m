function [ ] = initialze_params( )
%INITIALZE_PARAMS 此处显示有关此函数的摘要
%   此处显示详细说明
global timestep td t0 t
global Ixx Iyy Izz Jtp ct cq l m g h Kr Kparas I omegaMax omegaMin
global noise_rate
global allocation_matrix allocation_matrix_inv
global KpZ KiZ KdZ KpPhi KiPhi KdPhi KpTheta KiTheta KdTheta KpPsi KiPsi KdPsi KpX KiX KdX KpY KiY KdY gamma
global x_ xdot_ omega_
global acc_error acc_error_p


 
%% initialize system params
Ixx = 0.0095;  % Quadrotor moment of inertia around X axis
Iyy = 0.0095;  % Quadrotor moment of inertia around Y axis
Izz = 0.0186;  % Quadrotor moment of inertia around Z axis
Jtp = 3.7882e-06;  % Total rotational moment of inertia around the propeller axis
ct = 1.4865e-07;  % Thrust factor
cq = 8.06428*10^(-5);  % Drag factor
l = 0.2223;  % Distance to the center of the Quadrotor
m = 1.0230;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration
I = diag([Ixx Iyy Izz]); % inertia matrix\
omegaMax = 8976;
omegaMin = 1376;

h = 0.03;
Kr = 1e-03;
Kparas = 1e-03;
noise_rate=0;



allocation_matrix_inv = ...
    [ct, ct, ct, ct; ...
    0, l*ct, 0, -l*ct;...
    -l*ct, 0 l*ct, 0; ...
    -cq, cq, -cq, cq];

allocation_matrix = inv(allocation_matrix_inv);

%% PID params
KpZ = 100;
KiZ = 0.3;
KdZ = 40;

KpPhi = 100;
KiPhi = 0.04;
KdPhi = 50;

KpTheta = 100;
KiTheta = 0.04;
KdTheta = 50;

KpPsi = 20;
KiPsi = 0.2;
KdPsi = 20;

KpX = 0.5;
KiX = 0.015;
KdX = 0.7;
KpY = 0.5;
KiY = 0.015;
KdY = 0.7;

gamma = 0.9;

acc_error = zeros(12,1);
acc_error_p = zeros(4,1);

display('finish initializing system params');

end

