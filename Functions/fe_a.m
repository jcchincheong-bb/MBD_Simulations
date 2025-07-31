function actuator = fe_a(f_a,d)
    l = norm(d);  u = d/l;
    actuator = f_a*u;
end