function [xdot] = test_attitude(t,x,xd)
%% To test attitude pid response
%
global omega_ xdot_ x_
global acc_error

digits(6);

%% record intermediate x_
for i = 7:2:11
    x(i) = mod(x(i)+ pi, 2*pi) - pi;
end
x_ = [x_ x];

%% controller
[omega,acc_error] = pid_attitude_controller(x, xd, acc_error);
omega_ = [omega_ omega];

%% plant system
xdot = system_dynamics(x,omega);
xdot_ = [xdot_ xdot];

end