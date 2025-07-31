function Gamma_Tran = tran_gamma(r_i,r_j,u_j,rd_i,rd_j,phid_i)
% This function creates the gamma equation for a translation joint at
% point P and Q between bodies i and j. The main translation axis should be
% defined via body j
% Parameters are the position of the bodies (r), the unit vector of the
% main translation axis (u_j), the velocities of the bodies (rd) and the
% angular velocity of body i (phid_i)

    Gamma_Tran = [(s_rot(u_j)'*(r_i - r_j)*phid_i + 2*u_j'*(rd_i - rd_j))*phid_i;...
                0];
end