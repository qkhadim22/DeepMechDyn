function [Qe_12,Qe_2, s2, dots2] = Force_calculation_tilt_Cylinder(R1,phi1,R2,phi2,dR1,dphi1,dR2,dphi2,p3,p4,A1,A2,Fs,Fc,vs,sig2,pos, Cl)

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
R_ex12 = R1 + Rot_A1*pos.bar_hy2_up_1;
dR_ex12 = dR1 + dphi1*Rot_A1*I_til*pos.bar_hy2_up_1;
L_e12 = [I_2 Rot_A1*I_til*pos.bar_hy2_up_1];

%%%%% End on ground 2
Rot_A2 = [cos(phi2) -sin(phi2);
          sin(phi2) cos(phi2)];
R_ex2 = R2 + Rot_A2*pos.bar_hy2_up_2;
dR_ex2 = dR2 + dphi2*Rot_A2*I_til*pos.bar_hy2_up_2;
L_e2 = [I_2 Rot_A2*I_til*pos.bar_hy2_up_2];

%%%%% s and ds
s2 = sqrt((R_ex2-R_ex12)'*(R_ex2-R_ex12));
dots2 = (R_ex2-R_ex12)'*(dR_ex2-dR_ex12)/s2;
e_dir2 = (R_ex2-R_ex12)/s2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Actuator forces ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Friction forces in cylinders
F_mu_2= (Fc*tanh(4*(abs(dots2)/vs))+(Fs-Fc)*((abs(dots2)/vs)/.....
    ((1/4)*(abs(dots2)/vs)^2+3/4)^2))*sign(dots2)+sig2*dots2*tanh(4);

Fs2     = p3*A1-p4*A2-0*F_mu_2; % Hydraulic force

%%%%%  Actuator forces in Cylinders
% if abs(p3-p4) > 100e5
%      Fs2     = p3*A1-p4*A2-Cl *(p3*A1-p4*A2)-F_mu_2; % Hydraulic force
% else
%       Fs2     = p3*A1-p4*A2-F_mu_2; % Hydraulic force
% end 

F2 = Fs2*e_dir2;

%%%%%  Generalized global actuator forces
Qe_2 = L_e2'*F2;
Qe_12 = -L_e12'*F2;