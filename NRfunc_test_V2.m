m = 2;
syms x [m 1]               % syms variable array
F(x) = [-2*cos(x1)+3*cos(x2);10*sin(x1)+15*sin(x2)-18];   % function array (test from Num Math Chp 6)
x_in = [0.59;0.99]; % initial guess
n_max = 10; % step timeout
tol = 1e-15;  % tolerance for successful zero

[n,xout] = NRfunc(F,x,x_in,tol,n_max);
fprintf("In %d steps\n",n)
disp(xout)
