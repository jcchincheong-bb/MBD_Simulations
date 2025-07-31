%% System Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;
m = 3;
syms t
syms O [m 1]
syms Od [m 1]
syms Odd [m 1]

%% Equations
f_driver(t) = pi/2 + 2*pi*t;
phi_driver(t) = O1 - f_driver;
phi_con(O) = [l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a; ...
    l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b];


%% Solution Parameters 
time = 0;
phi(O) = [phi_con; phi_driver(time)];
D = jacobian(phi,O);
O_in = [pi/2;0.58;-1.8];  % initial guess
n_max = 10;             % max steps
tol = 1e-15;  % tolerance for successful zero

%% Solution
% Position
[n,O_sol] = NRfunc(phi,O,O_in,tol,n_max);
fprintf("In %d steps\n",n)
fprintf("The positions\n")
disp(O_sol)

% Velocity
rhsv = zeros([m,1]);
fd = diff(f_driver);
rhsv(m) = fd(time);
D_sol = subs(D,O,O_sol);   % update the jacobian 
Od_sol = double(D_sol\rhsv);
fprintf("The velocities\n")
disp(Od_sol)

% Acceleration
fdd = diff(fd);
