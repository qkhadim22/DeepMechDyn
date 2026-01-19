function [Qe_1, s1, dots1] = Force_calculation_lift_Cylinder(t, R1,phi1,dR1,dphi1,p1,p2,A1,A2,Fs,Fc,vs,sig2,pos,Cl)

I_2 = [1 0;
       0 1];
I_til = [0 -1;
         1 0];
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Caculate the s and dots *****
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% End on body 1
Rot_A1 = [cos(phi1) -sin(phi1);
          sin(phi1) cos(phi1)];
R_ex1 = R1 + Rot_A1*pos.bar_hy1_up_1;
dR_ex1 = dR1 + dphi1*Rot_A1*I_til*pos.bar_hy1_up_1;

%%%%% End on ground 0
R_ex0 = pos.bar_hy1_up_0;
dR_ex0 = [0; 0];

%%%%% s and ds
s1 = sqrt((R_ex1-R_ex0)'*(R_ex1-R_ex0));
dots1 = (R_ex1-R_ex0)'*(dR_ex1-dR_ex0)/s1;
e_dir1 = (R_ex1-R_ex0)/s1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Actuator forces ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Friction forces in cylinders
F_mu_1= (Fc*tanh(4*(abs(dots1)/vs))+(Fs-Fc)*((abs(dots1)/vs)/.....
    ((1/4)*(abs(dots1)/vs)^2+3/4)^2))*sign(dots1)+sig2*dots1*tanh(4);

%%%%%  Actuator forces in Cylinders
if abs(p1-p2) > 100e5
     Fs1     = p1*A1-p2*A2-Cl *(p1*A1-p2*A2)-F_mu_1; % Hydraulic force
else
      Fs1     = p1*A1-p2*A2-F_mu_1; % Hydraulic force
end 

% if abs(p1-p2) > 100e5
%      Fs1     = p1*A1-p2*A2-Cl *(p1*A1-p2*A2)-F_mu_1; % Hydraulic force
% else
%       Fs1     = p1*A1-p2*A2-F_mu_1; % Hydraulic force
% end 

F1 = Fs1*e_dir1;

%%%%%  Generalized global actuator forces

L_e1 = [I_2 Rot_A1*I_til*pos.bar_hy1_up_1]; %???????
Qe_1 = L_e1'*F1;