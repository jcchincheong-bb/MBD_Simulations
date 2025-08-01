%% Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;
O1 = pi/2; % for t=0 in phi3

%% Equations
% phi = @(O)([l1*cos(O(1))+l2*cos(O(2))+l3*cos(O(3))-a; ...
%     l1*sin(O(1))+l2*sin(O(2))+l3*sin(O(3))-b]);
phi = @(O)([l1*cos(O1)+l2*cos(O(1))+l3*cos(O(2))-a; ...
    l1*sin(O1)+l2*sin(O(1))+l3*sin(O(2))-b]);

%% Solving using NR Multivariable Method (fsolve)
theta0 = [0;0]; % initial guess
fsolve(phi,theta0)