clear;
clc;
close all

global timestep td t0 t
global Ixx Iyy Izz Jtp b d l m g h Kr Kparas  I
global noise_rate
global allocation_matrix allocation_matrix_inv
global KpZ KdZ KpPhi KdPhi KpTheta KdTheta KpPsi KdPsi

%% initialize
initialze_params();

%% StartPoint & Destination
x0=zeros(12,1);
xd=zeros(12,1);

Zd = 10;
Phid = pi/12;
Thetad = 0;
Psid = pi/12;

% x0 = 0.1* ones(12,1);
x0(5) = 0; x0(9) = pi/3;
xd(5) = Zd; xd(7) = Phid; xd(9) = Thetad; xd(11) = Psid;

%% set simuation time
timestep = 0.01;
td = 5.00;
t = 0:timestep:td;
t0 = 0;

%% wind value
wind_con = ones(3, length(t)) .* 0;
wind_var = rand(3, length(t)) .* 0;
wind = wind_con + wind_var;
wind = [t;wind];


%% fill X with state
X = x0;

%% regulate angles
X(7) = mod(X(7)+pi, 2*pi) - pi;
X(9) = mod(X(7)+pi, 2*pi) - pi;
X(11) = mod(X(7)+pi, 2*pi) - pi;

%% calulate PID control signal
% U = zeros(4,1);    % The control vector
% 
% % use U(1) to complement Z error, with 
% U(1) = m*(g + Kpz*(Zd - X(5)) + Kdz*( - X(6)))/(cos(X(9))*cos(X(7)));   % Total Thrust on the body along z-axis
% U(2) = (Kpp*(Phid - X(7)) + Kdp*(0 - X(8)));   % Roll input
% U(3) = (Kpt*(Thetad - X(9)) + Kdt*( 0- X(10)));   % Pitch input
% U(4) = (Kpps*(Psid - X(11)) + Kdps*( 0- X(12)));   % Yawing moment
% 
% if U(1) > 10.9264
%     U(1) = 10.9264;
% end
% if U(1) < 0
%     U(1) = 0;
% end
% 
% for j = 2:4
%     if U(j) > 0.1
%         U(j) = 0.1;
%     end
%     
%     if U(j) < -0.1
%         U(j) = -0.1;
%     end
% end

%% initialize forces and torques
forces = zeros(3,1);
torques = zeros(3,1);

% calculate rotational matrix
phi = X(7); theta = X(9); psi=X(11);
dphi = X(8); dtheta = X(10); dpsi = X(12);

Rib = angle2dcm(psi,theta,phi);
Rbi = Rib';

%% from control input calculate omega
omega_square = allocation_matrix * U;
omega0 = sqrt(omega_square);
% omega_square(omega_square > 523^2) = 523^2;
omega_square(omega_square > 550^2) = 550^2;
omega_square(omega_square < 125^2) = 125^2;
omega = sqrt(omega_square);

%% calculate thrust
forces = zeros(3,1);
torques = zeros(3,1);

% calculate thrust
omega_square = omega.^2; % squared omega
thrust = b * omega_square;

forces(3) = forces(3) + sum(thrust);

torques(1) = torques(1) + Jtp .* dtheta .* [1,-1,1,-1] * omega;  %roll --gyro
torques(2) = torques(2) + Jtp .* dphi .* [-1,1,-1,1] * omega;

torques(1)= torques(1) + b*l*(omega_square(2)-omega_square(4));
torques(2)= torques(2) + b*l*(omega_square(3)-omega_square(1));
torques(3)= torques(3) + d * [-1,1,-1,1] * omega_square;

%% caLculate state variation
acceleration = Rbi * (forces ./ m) - [0;0;g];

%% test
UUUU = generate_matrix *  omega_square; 


