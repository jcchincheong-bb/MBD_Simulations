function A = A_matrix(phi)
% This function computes the coordinate rotation matrix for a given angle
% phi
    cp = cos(phi); sp = sin(phi); A = [cp -sp; sp cp];
end