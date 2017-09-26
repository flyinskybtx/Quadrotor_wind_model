function [ xdot ] = full_dynamics( t,x,xd )
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
global omega_
global xdot_
global x_
global acc_error acc_error_p
global wind

digits(6);

% get wind at time t
wind_t=zeros(3,1);
wind_t(1) = interp1(wind(1,:), wind(2,:),t);
wind_t(2) = interp1(wind(1,:), wind(3,:),t);
wind_t(3) = interp1(wind(1,:), wind(4,:),t);

% figure(1)
% hold on
% plot(t,wind_t(1),'ro');
% hold off;

for i = 7:2:11
    x(i) = mod(x(i)+ pi, 2*pi) - pi;
end
x_ = [x_ x];

%% position controller
[xd_new,acc_error_p] = pid_position_controller(x,xd, acc_error_p);
% xd_new = xd;

%% controller
[omega,acc_error] = pid_attitude_controller(x, xd_new, acc_error);
omega_ = [omega_ omega];
%% plant system
xdot = system_dynamics(x,omega,wind_t);
xdot_ = [xdot_ xdot];



end

