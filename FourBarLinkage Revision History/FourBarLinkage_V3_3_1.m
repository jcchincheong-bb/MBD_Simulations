%%%%%%%%%%%%%%%%%%%%%%%%
% Description : Script to solve the four bar linkage problem using 
% Naive Syms Approach
%%%%%%%%%%%%%%%%%%%%%%%%
%% Reset
clear
%% System Parameters 
a = 2.0;
b = 0.5;
l1 = 1.0;
l2 = 3.0;
l3 = 2.2;
m = 3;
syms t
syms O [m 1]
syms Od [m 1]
syms Odd [m 1]

%% Position
% Position Constraint Equations
phi = [l1*cos(O1)+l2*cos(O2)+l3*cos(O3)-a==0;...
    l1*sin(O1)+l2*sin(O2)+l3*sin(O3)-b==0;...
    O1-pi/2==0]; 
% Position Solution
S = solve(phi,O);
O1=S.O1(1); O2=S.O2(2); O3=S.O3(1);  % Updating Values of Position
fprintf("\nO1:") 
disp(vpa(O1)) 
fprintf("\nO2:") 
disp(vpa(O2))
fprintf("\nO3:") 
disp(vpa(O3))


%% Velocity
% Velocity Constraint Equations
phid = [-l1*sin(O1)*Od1-l2*sin(O2)*Od2-l3*sin(O3)*Od3==0;...
    l1*cos(O1)*Od1+l2*cos(O2)*Od2+l3*cos(O3)*Od3==0;...
    Od1-2*pi==0];

% Velocity Solution
Sd = solve(phid,Od);
Od1=Sd.Od1(1); Od2=Sd.Od2(1); Od3=Sd.Od3(1);
fprintf("\nOd1:") 
disp(vpa(Od1))
fprintf("\nOd2:") 
disp(vpa(Od2))
fprintf("\nOd3:") 
disp(vpa(Od3))


% %% Accleration
% Acceleration Constraint Equations
phidd = [-l1*cos(O1)*Od1^2-l1*Odd1*sin(O1)-l2*cos(O2)*Od2^2-l2*sin(O2)*Odd2-l3*cos(O3)*Od3^2-l3*sin(O3)*Odd3==0;...
    l1*cos(O1)*Odd1-l1*sin(O1)*Od1^2+l2*cos(O2)*Odd2-l2*sin(O2)*Od2^2+l3*cos(O3)*Odd3-l3*sin(O3)*Od3^2==0;...
    Odd1==0];

% Acceleration Solution
Sdd = solve(phidd,Odd);
Odd1=Sdd.Odd1(1); Odd2=Sdd.Odd2(1); Odd3=Sdd.Odd3(1);
fprintf("\nOdd1:") 
disp(vpa(Odd1))
fprintf("\nOdd2:") 
disp(vpa(Odd2))
fprintf("\nOdd3:") 
disp(vpa(Odd3))