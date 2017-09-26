clear;
clc;
close all

% global timestep td t0 t
global Ixx Iyy Izz Jtp ct d l m g h Kr Kparas  I omegaMax omegaMin 
global noise_rate
global allocation_matrix allocation_matrix_inv
global KpZ KiZ KdZ KpPhi KiPhi KdPhi KpTheta KiTheta KdTheta KpPsi KiPsi KdPsi KpX KiX KdX KpY KiY KdY gamma
global x_ xdot_ omega_
global acc_error acc_error_p
global wind windCon windVar

digits(6);

%% initialize
initialze_params();

%% StartPoint & Destination, used for test
x0=zeros(12,1);
xd=zeros(12,1);

Zd = 10;
Phid = pi/6 ;
Thetad = pi/6;
Psid = -pi/6;

%set x0
x0(5) = 10;

%set xd
xd(5) = Zd; xd(3) = 3; xd(1) = 5; % position controller test 
% xd(7)  = Phid; xd(9) = Thetad; xd(11) = Psid; % attitude controller test

% global evaluation
x_ =[];
xdot_ = [];
omega_ = [];

% test
xd1 = xd;
xd1(9) = 0;
xd2 = xd;
xd2(7) = 0;

%% set simuation time
timestep = 0.01;
td = 10;
t = 0:timestep:td;
t0 = 0;

%% wind value
windCon = 10;
windVar = 0;
wind_con = [ones(2, length(t)-700) .* windCon; zeros(1,length(t)-700)];
wind_con = [zeros(3,700) wind_con];
wind_var = rand(3, length(t)) .* windVar;
wind = wind_con + wind_var;
wind = [t;wind];

%% solve ode
tic

ode_options = odeset('RelTol',1e-4,'AbsTol',[1e-5*ones(1,6) 1e-4*ones(1,6)]); 
[tt,x] = ode45(@full_dynamics, [t(1) t(end)], x0,ode_options,xd);  
x=x'; tt= tt';

toc

%% display
xd(5) = Zd; xd(7)  = Phid; xd(9) = Thetad; xd(11) = Psid;
display_(tt,x,xd);

% 
% %test
% [t1,x1] = ode45(@full_dynamics, [t(1) t(end)], x0,ode_options,xd1); 
% x1=x1'; t1= t1';
% display_(t1,x1,xd1);
% 
% [t2,x2] = ode45(@full_dynamics, [t(1) t(end)], x0,ode_options,xd2);
% x2=x2'; t2= t2';
% display_(t2,x2,xd2);