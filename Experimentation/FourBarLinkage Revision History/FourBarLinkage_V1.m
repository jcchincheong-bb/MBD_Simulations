%% Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;

%% Equations
phi1 = @(O1,O2,O3)(l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a);
phi2 = @(O1,O2,O3)(l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b);
phi3 = @(O1,t)(O1-pi/2-2*pi*t);
fO1 = @(t)(pi/2 + 2*pi*t);  % function to find O1 at given time t 

%% Solving using NR Multivariable method (scratch)
O1 = pi/2; % for t=0 in phi3
theta0 = [O1;0.4;0.6]; % initial guess
theta1 = theta0 - inv(jacobian(theta0(1),theta0(2),theta0(3)))*[phi1(theta0(1),theta0(2),theta0(3));phi2(theta0(1),theta0(2),theta0(3));phi3(theta0(1),0)];
theta1
%% Functions
function [D] = jacobian(O1,O2,O3)
	l1 = 1.0;
    l2 = 3.0;
    l3 = 2.2;
    D = [-l1*sin(O1) -l2*sin(O2) -l3*sin(O3);l1*cos(O1) l2*cos(O2) l3*cos(O3);1 0 0];
end