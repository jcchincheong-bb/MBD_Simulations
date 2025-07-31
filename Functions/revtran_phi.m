function Phi_tr = revtran_phi(ri,rj,sPi,sQj,u,L)
% This function creates the constraint equation for a rev-tran joint at
% point P and Q between bodies i and j.
% Parameters are the position of the body (r) and displacement of the
% points (s), the unit vector of the main translation axis on body j (u) 
% and the fixed distance between the points (L).      
    d = (ri+sPi) - (rj+sQj);
    Phi_tr = s_rot(u)'*d - L;
end