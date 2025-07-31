syms r  [1 3]  matrix
syms t 
syms v(r,t)  [1 2]  matrix  keepargs
v(r,t) = [r^2;sin(r)]
vEval = v([1 2 2],3)
vSym = symmatrix2sym(vEval)