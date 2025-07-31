function r_P_d = r_Point_d(r_d, s_P, phi_d)
% This function finds the global velocity of a point r_P_d [m/s] using the body global
% velocities r_d [m/s] and phi_d [rad/s] and the global displacement s_P
% [m]
 r_P_d = r_d + s_rot(s_P)*phi_d; 
end