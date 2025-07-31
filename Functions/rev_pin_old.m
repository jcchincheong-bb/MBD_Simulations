function PhiP = rev_pin_old(ri,rj,phii,phij,sPi_l,sPj_l)
% This function creates the constraint equation for a revolute joint at
% point P betwee bodies i and j
    PhiP = ri + A_matrix(phii)*sPi_l - (rj + A_matrix(phij)*sPj_l);
end