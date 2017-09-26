function [ xd_new,acc_error ] = pid_position_controller( x,xd,acc_error )
%UNTITLED5 此处显示有关此函数的摘要
%   x xd
global  KpX KiX KdX KpY KiY KdY
global Ixx Iyy Izz Jtp ct cq l m g h Kr Kparas omegaMax omegaMin
global allocation_matrix allocation_matrix_inv
global displayPhid displayThetad
global gamma

digits(6);

%% variables
psi = x(11);

error = xd - x;
error = error(1:4); % only the horizonal translation error


Rib = [cos(psi) 0 sin(psi) 0; 0 cos(psi) 0 sin(psi);-sin(psi) 0 cos(psi) 0; 0 -sin(psi) 0 cos(psi)];
error_b = Rib * error;
acc_error = gamma .* acc_error + error;
acc_b = Rib * acc_error;


%% Position controller


Thetad = [KpX KdX] * error_b(1:2) + KiX * acc_b(1); % negative Theta generate positive X 
Phid = -1 .* ( [KpY KdY] * error_b(3:4) + KiY * acc_b(3));

% display([Phid Thetad])

if abs(Thetad) > pi/4 % regulate desire angle
    Thetad =  pi/4 * sign(Thetad);
end

if abs(Phid) > pi/4
    Phid = pi/4 * sign(Phid);
end

xd_new = xd;
xd_new(7) = Phid;
xd_new(9) = Thetad;

%% test
% xd_new(7) = 0;
% xd_new(9) = 0;

end

