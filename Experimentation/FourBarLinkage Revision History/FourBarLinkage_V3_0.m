%% Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;
n= 4;
syms t
syms O [n 1] matrix

%% Equations
syms phi(O) [2 1] matrix keepargs
phi(O) = [l1*cos(O(1))+l2*cos(O(2))+l3*cos(O(3))-a; ...
    l1*sin(O(1))+l2*sin(O(2))+l3*sin(O(3))-b];

D = diff(phi,O)