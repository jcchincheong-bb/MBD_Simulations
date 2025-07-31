function spring = fe_s(ks,l0,d)
    l = norm(d);  u = d/l;
    f_s = ks*(l-l0);
    spring = f_s*u;
end