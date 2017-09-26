function [ xdot ] = system_dynamics( x,omega,wind,t )
%UNTITLED4 此处显示有关此函数的摘要
%   x, omega
global Ixx Iyy Izz Jtp ct cq l m g h Kr Kparas I omegaMax omegaMin

digits(6);


%% regulate state and control input
if x(5) < 0 % no lower than ground, if hit ground reinitialize;
    x(:,:) =0;
end

omega(omega>omegaMax) = omegaMax; % bound omega
omega(omega<omegaMin) = omegaMin;

phi = x(7); theta = x(9); psi=x(11);
dphi = x(8); dtheta = x(10); dpsi = x(12);
q = [dphi; dtheta; dpsi];

%% calculate rotational matrix
Rib = angle2dcm(psi,theta,phi);%  rotational matrix
Rbi = Rib';
Rq = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); ...
    0 cos(phi) -sin(phi); ...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

%% calculate free stream velocity
vinf = [x(2);x(4);x(6)] - wind;
vinf_b = Rib * vinf; % free stream in B
vh = [vinf_b(1:2); 0]; % vh is horizonal terms of vinf_b

%% calculate thrust
omega_square = omega.^2; % squared omega
thrust = ct * omega_square;

%% calculate wind effects
%% calculate drag
drag = zeros(4,3);
for j=1:4
    drag(j,:) =  Kr .* thrust(j) .* vh';% lumped model  of blade flapping, induced drag, profile drag, translational drag
    drag(j,:) = drag(j,:) + Kparas .* thrust(j) .* norm(vinf) .* vinf'; %parasitic drag
end

%% calculate drag moment
torque_drag = zeros(4,3);
torque_drag(1,:) =  cross(drag(1,:), [l,0,0]-[0,0,h]);
torque_drag(2,:) =  cross(drag(2,:), [0,l,0]-[0,0,h]);
torque_drag(3,:) =  cross(drag(3,:), -[l,0,0]-[0,0,h]);
torque_drag(4,:) =  cross(drag(4,:), -[l,l,0]-[0,0,h]);

%% calculate forces and torques
% sum up drag forces and torques
forces = sum(drag);
forces = forces';
torques = sum(torque_drag);
torques = torques';

forces(3) = forces(3) + sum(thrust); % thrust
forces = forces + Rib * [0;0; -m*g];

torques(1) = torques(1) + Jtp .* dtheta .* [1,-1,1,-1] * omega;  % roll --gyro
torques(2) = torques(2) + Jtp .* dphi .* [-1,1,-1,1] * omega;
torques(1)= torques(1) + ct*l*(omega_square(2)-omega_square(4)); % roll --thrust
torques(2)= torques(2) + ct*l*(omega_square(3)-omega_square(1));
torques(3)= torques(3) + cq * [-1,1,-1,1] * omega_square;

%% calculate dynamic ode
acceleration = Rbi * (forces ./ m) ; % translational acceleration
dq =  I\torques + I\cross(q, I * q); % rotational acceleration

rate = Rq * [x(8);x(10);x(12)]; % body rotational rate

xdot = zeros(12,1); % column vector
xdot(1)= x(2);
xdot(3)= x(4);
xdot(5)= x(6);

xdot(7)= rate(1);
xdot(9)= rate(2);
xdot(11)= rate(3);

xdot(2) = acceleration(1);
xdot(4) = acceleration(2);
xdot(6) = acceleration(3);

xdot(8) = dq(1);
xdot(10) = dq(2);
xdot(12) = dq(3);

end

