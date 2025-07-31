function Phi_Rev = rev_pin_phi(ri,rj,sPi,sPj)
% This function creates the constraint equation for a revolute joint at
% point P between bodies i and j.
% Parameters are the position of the body (r) and displacement of the point
% (s) in the global reference frame
    Phi_Rev = ri + sPi - (rj + sPj);
end