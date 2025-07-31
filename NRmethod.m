%% Equation 
syms y(x) 
y(x) = x^3 -3*x^2 - 6*x + 8;
dy = diff(y,x);

%% Parameters
x1 = input('What is your guess?'); % intial guess
n = 0; % number of steps
i = 1; % iteration variable
n_max = 100;
tol = 0.00000000000001;

%% Solving 
x_sol = zeros([1 1]);
x_sol(1) = x1;

while true
    if (abs(y(x_sol(i)))<=tol)
        break
    elseif (n == n_max)
        printf("Max Iteration Steps Exceeded")
        break
    else    
        x_sol(i+1) = x_sol(i) + (-1/dy(x_sol(i)) * y(x_sol(i)));
        n = n+1;
    end
    i = i+1;
end
y_sol = y(x_sol);
dy_sol = dy(x_sol);

%% Table
T = table([0:n]',vpa(x_sol)',vpa(y_sol)',vpa(dy_sol)','VariableNames',{'j','xj','yj','dyj'});
disp(T)