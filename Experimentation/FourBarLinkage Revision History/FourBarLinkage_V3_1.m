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
phi_d = O1 - pi/2
phi(O) = [l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a; ...
    l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b;...
    O1-pi/2];

%% Solution Parameters 
O_in = [pi/2;0.58;-1.8];  % initial guess
n_max = 10;             % max steps
tol = 1e-15;  % tolerance for successful zero

%% Solution
[n,xout] = NRfunc(phi,O,O_in,tol,n_max);
fprintf("In %d steps\n",n)
disp(xout)
 
