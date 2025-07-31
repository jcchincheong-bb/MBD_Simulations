%% Reset
clear;   % resets all the variables since syms can be a bit fucky wucky

%% Equations 
m = 2;                     % number of variables 
syms x [m 1]               % syms variable array
F(x) = [-2*cos(x1)+3*cos(x2);10*sin(x1)+15*sin(x2)-18];   % function array (test from Num Math Chp 6)
D = jacobian(F,x);       % Jacobian matrix

%% Parameters
% x_in = input('What is your guess?'); % intial guess
x_in = [0.59;0.99];
n = 0; % number of steps
n_max = 10; % step timeout
tol = 1e-15;  % tolerance for successful zero

%% Solving 
x_sol = zeros([m 1]);
x_sol = x_in;              % setting the initial guess 
f_out = subs(F,x,x_sol(:,1));    % evaluating the inital guess function value
df_out = subs(D,x,x_sol(:,1));   % evaluating the inital guess jacobian value
ep = sum(double(f_out));         % error
while (abs(ep)>=tol) && (n<n_max)
    x_sol(:,n+2) = x_sol(:,n+1) - (inv(df_out) * f_out);    % NR Method multivariable 
    f_out = subs(F,x,x_sol(:,n+2));                         % update function value
    df_out = subs(D,x,x_sol(:,n+2));                        % update jacobian
    ep = sum(double(f_out));                                % update the error
    n = n+1;
end
x_out = vpa(x_sol)