function Phi_Tran = tran_phi(ri,rj,ui,uj,sPi,sQj)
% This function creates the constraint equation for a translation joint at
% point P and Q between bodies i and j. The main translation axis should be
% defined via body j, though theorectically it does not matter
% Parameters are the position of the body (r) and displacement of the point
% (s) in the global reference frame and the unit vectors of translation axes of each body    
    Phi_Tran = [s_rot(uj)'*ui;...
                s_rot(uj)'*(ri + sPi - (rj + sQj))];
end