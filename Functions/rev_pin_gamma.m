function Gamma_Rev = rev_pin_gamma(phidi,phidj,sPi,sPj)
% This function creates the acceleration gamma function for a revolute joint at
% point P between bodies i and j.
% Parameters are the angular velocity (phid) and displacement of the point
% (s) in the global reference frame
    Gamma_Rev = sPi*(phidi)^2 - sPj*(phidj)^2;
end