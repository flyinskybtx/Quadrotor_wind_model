function [ ] = display_(t, x,xd)
%UNTITLED8 此处显示有关此函数的摘要
%   此处显示详细说明
if nargin == 2
    xd = zeros(12,1);
end

xd = xd * ones(1,length(x(1,:)));

global x_ xdot_ omega_

figure;
hold on
plot(t,x(1,:),'r');
plot(t,x(3,:),'g');
plot(t,x(5,:),'b');

plot(t,xd(1,:),'r--');
plot(t,xd(3,:),'g--');
plot(t,xd(5,:),'b--');
hold off;

figure;
hold on
plot(t,x(7,:),'r');
plot(t,x(9,:),'g');
plot(t,x(11,:),'b');

plot(t,xd(7,:),'r--');
plot(t,xd(9,:),'g--');
plot(t,xd(11,:),'b--');
hold off;

end

