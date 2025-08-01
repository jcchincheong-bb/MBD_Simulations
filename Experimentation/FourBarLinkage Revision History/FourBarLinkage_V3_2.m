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
phi(O) = [l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a; ...
    l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b;...
    O1-pi/2];

%% Solution Parameters 
O_in = [pi/2;0.4;0.6];  % initial guess
n_max = 10;             % max steps
tol = 1e-15;  % tolerance for successful zero

%% Solution
D = jacobian(phi,O);                        % Jacobian matrix
x_sol = O_in;                             % setting the initial guess 
f_out = subs(phi,O,x_sol);             % evaluating the inital guess function value
df_out = subs(D,O,x_sol);            % evaluating the inital guess jacobian value
ep = sum(double(f_out));                  % error
for n=1:4
    x_sol= x_sol - (inv(df_out) * f_out); % NR Method multivariable 
    f_out = subs(phi,O,x_sol);              % update function value
    df_out = subs(D,O,x_sol);             % update jacobian
    ep = sum(double(f_out));              % update the error
end
xout = vpa(x_sol);                       % send output
 
