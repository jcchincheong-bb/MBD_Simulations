function damper = fe_d(kd,d,d_d)
    l = norm(d);  u = d/l;
    l_d = (d_d' * d)/l;
    f_d = kd*l_d;
    damper = f_d*u;
end