function Gamma_tr = revtran_gamma(ri,rj,rdi,rdj,phidi,phidj,sPi,sQj,u)
% This function creates the gamma equation for a rev-tran joint at
% point P and Q between bodies i and j.
% Parameters are the position of the body (r), velocities of the body (rd, phid), 
% the unit vector of the main translation axis on body j (u) and displacement of the points (s).      
    d = (ri+sPi) - (rj+sQj);
    dd = (rdi - rdj + s_rot(sPi)*phidi - s_rot(sQj)*phidj);
    Gamma_tr = phidi*s_rot(u)'*(d*phidi+2*s_rot(dd)) + u'*(sPi*phidi^2 - sQj*phidj^2);
end