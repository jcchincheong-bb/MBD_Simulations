function [xout,n] = NRfunc(F,x,x_in,tol,n_max)
    D = jacobian(F,x);                        % Jacobian matrix
    x_sol = x_in;                             % setting the initial guess 
    flag = 0;                                 % variable to see if convergence is met
    for n=1:n_max
        f_out = vpa(subs(F,x,x_sol));         % evaluating the function value
        df_out = vpa(subs(D,x,x_sol));        % evaluating the jacobian value
        ep = norm(double(f_out));             % error
        if (abs(ep)<tol)
            flag = 1;                         % convergence is met
            break
        end
        dx_sol= vpa(-df_out\f_out);           % finding the correction
        x_sol = x_sol + dx_sol;               % correcting the value
    end
    
    if (flag == 0)
        fprintf("Convergence not met, step limit reached.")
    end

    xout = double(x_sol);                       % send output
end