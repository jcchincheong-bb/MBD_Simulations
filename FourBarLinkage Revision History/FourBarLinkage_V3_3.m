%% System Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;
m = 3;
syms t
syms O [m 1]

%% Equations
phi = [l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a==0; ...
    l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b==0;...
    O1-pi/2==0];

%% Solution
S = solve(phi,O);
num_sol_1 = double([S.O1(1);S.O2(1);S.O3(1)])
num_sol_2 = double([S.O1(2);S.O2(2);S.O3(2)])