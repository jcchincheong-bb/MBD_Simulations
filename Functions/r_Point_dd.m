function r_P_dd = r_Point_dd(r_dd, s_P, phi_d,phi_dd)
% This function finds the global acceleration of a point r_P_dd [m/s^2] using the body global
% accelerations r_dd [m/s^2] and phi_dd [rad/s^2], the body global angular velocity phi_d [rad/s] 
% and the global displacement s_P [m]
    r_P_dd = r_dd + s_rot(s_P)*phi_dd - s_P*(phi_d)^2; 
end