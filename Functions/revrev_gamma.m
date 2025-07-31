function Gamma_rr = revrev_gamma(ri,rj,rdi,rdj,phidi,phidj,sPi,sQj)
% This function creates the gamma equation for a rev-rev joint at
% point P and Q between bodies i and j.
% Parameters are the position of the body (r), velocities of the body (rd, phid), 
% and displacement of the points (s).  
    d = (ri+sPi) - (rj+sQj);
    dd = (rdi - rdj + s_rot(sPi)*phidi - s_rot(sQj)*phidj);
    Gamma_rr = -dd'*dd - d'*(-sPi*phidi^2 + sQj*phij^2);
end