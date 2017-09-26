function [ omega,acc_error ] = pid_attitude_controller( x,xd, acc_error)
%UNTITLED5 此处显示有关此函数的摘要
%   x xd
global KpZ KiZ KdZ KpPhi KiPhi KdPhi KpTheta KiTheta KdTheta KpPsi KiPsi KdPsi
global Ixx Iyy Izz Jtp ct cq l m g h Kr Kparas omegaMax omegaMin
global allocation_matrix allocation_matrix_inv
global gamma

if nargin == 2
    gamma = 0;
end

%% variables
Zd = xd(5);
Phid = xd(7);
Thetad = xd(9);
Psid = xd(11);

phi = x(7);
theta = x(9);
psi = x(11);

error = xd - x;
acc_error = gamma .* acc_error + error;



%% calulate attitude PD control signal
U = zeros(4,1);    % The control vector
% use U(1) to complement Z error,
U(1) = (m * g + m * [KpZ KdZ] * error(5:6)) / (cos(phi) * cos(theta)) + KiZ * acc_error(5);
U(2) = Ixx * [KpPhi KdPhi] * error(7:8) +  KiPhi * acc_error(7);
U(3) = Iyy * [KpTheta KdTheta] * error(9:10) + KiTheta * acc_error(9);
U(4) = Izz * [KpPsi KdPsi] * error(11:12) + KiPsi * acc_error(11);

if U(1) > 20
    U(1) = 20;
elseif U(1) < 0
    U(1) = 0;
end
for j = 2:4
    if abs(U(j)) > 0.1
        U(j) = 0.1 * sign(U(j));
    end
end

%% translate into omega
omega = zeros(4,1);
omega_square = allocation_matrix * U;
omega =  sqrt(omega_square);

omega(omega>omegaMax) = omegaMax;
omega(omega<omegaMin) = omegaMin;

end

