clear;
clc;
close all

% global timestep td t0 t
global Ixx Iyy Izz Jtp ct d l m g h Kr Kparas I omegaMax omegaMin 
global noise_rate
global allocation_matrix allocation_matrix_inv
global KpZ KiZ KdZ KpPhi KiPhi KdPhi KpTheta KiTheta KdTheta KpPsi KiPsi KdPsi gamma
global x_ xdot_ omega_
global acc_error

digits(6);

%% initialize
initialze_params();
KpZ = 100;
KiZ = 0.3;
KdZ = 40;

KpPhi = 100;
KiPhi = 0.04;
KdPhi = 50;

KpTheta = 30;
KiTheta = 0.04;
KdTheta = 30;

KpPsi = 60;
KiPsi = 0.02;
KdPsi = 20;

%% StartPoint & Destination, used for test
x0=zeros(12,1);
xd=zeros(12,1);

Zd = 10;
Phid = 0 ;
Thetad = pi/6;
Psid = 0;

%set x0
x0(5) = 10;

%set xd
xd(5) = Zd; xd(7)  = Phid; xd(9) = Thetad; xd(11) = Psid; % attitude controller test

% global evaluation
x_ =[];
xdot_ = [];
omega_ = [];

%% set simuation time
timestep = 0.01;
td = 10;
t = 0:timestep:td;
t0 = 0;

%% solve ode
tic

ode_options = odeset('RelTol',1e-4,'AbsTol',[1e-5*ones(1,6) 1e-4*ones(1,6)]); 
[tt,x] = ode45(@test_attitude, [t(1) t(end)], x0,ode_options,xd);  
x=x'; tt= tt';

toc

%% display
xd(5) = Zd; xd(7)  = Phid; xd(9) = Thetad; xd(11) = Psid;
display_(tt,x,xd);
