function Phi_rr = revrev_phi(ri,rj,sPi,sQj,L)
% This function creates the constraint equation for a rev-rev joint at
% point P and Q between bodies i and j.
% Parameters are the position of the body (r) and displacement of the
% points (s) and the fixed distance between the points (L).  
    d = (ri+sPi) - (rj+sQj);
    Phi_rr = d'*d - L^2;
end