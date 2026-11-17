function [Q_hyd, s1, dots1, s2, dots2, F_mu_1, F_mu_2,Fs1,Fs2] = Global_force_hydraulic(R,phi,dR,dphi,p_hyd, pos,param)
% New numbering: 1 lift boom, 2 tilt boom, 3 bracket 2, 4 bracket 1
% Cylinder 1 (lift): ground  -> body 1
% Cylinder 2 (tilt): body 1  -> body 4 (bracket 1)

pA1         = p_hyd(1);
pB1         = p_hyd(2);
pA2         = p_hyd(3);
pB2         = p_hyd(4);

I_2         = [1 0; 0 1];
I_til       = [0 -1;1 0];

function A = Rot_Matrix(phi)
    A = [cos(phi) -sin(phi);
          sin(phi) cos(phi)];
end 
                                                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Cylinder 1 (s1, dots1, Fh1) ***   base on ground, rod end on body 1 (lift boom)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R1          = R(:,1);
phi1        = phi(1);
dR1         = dR(:,1);
dphi1       = dphi(1);
R_ex1       = R1 + Rot_Matrix(phi1)*pos.bar_hy1_up_1;
dR_ex1      = dR1 + dphi1*Rot_Matrix(phi1)*I_til*pos.bar_hy1_up_1;
R_ex0       = pos.bar_hy1_up_0;
dR_ex0      = [0; 0];
s1          = sqrt((R_ex1-R_ex0)'*(R_ex1-R_ex0));
dots1       = (R_ex1-R_ex0)'*(dR_ex1-dR_ex0)/s1;
e_dir1      = (R_ex1-R_ex0)/s1;
[Fs1, F_mu_1]= Hydraulic_force(param,pA1,pB1, s1, dots1,1);
L_e1        = [I_2 Rot_Matrix(phi1)*I_til*pos.bar_hy1_up_1]; 
F1          = Fs1*e_dir1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Cylinder 2 (s2, dots2, Fh2) ***   base on body 1 (lift boom), rod end on body 4 (bracket 1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R4          = R(:,4);
phi4        = phi(4);
dR4         = dR(:,4);
dphi4       = dphi(4);
R_ex12      = R1 + Rot_Matrix(phi1)*pos.bar_hy2_up_1;
dR_ex12     = dR1 + dphi1*Rot_Matrix(phi1)*I_til*pos.bar_hy2_up_1;
R_ex4       = R4 + Rot_Matrix(phi4)*pos.bar_hy2_up_4;
dR_ex4      = dR4 + dphi4*Rot_Matrix(phi4)*I_til*pos.bar_hy2_up_4;
s2          = sqrt((R_ex4-R_ex12)'*(R_ex4-R_ex12));
dots2       = (R_ex4-R_ex12)'*(dR_ex4-dR_ex12)/s2;
e_dir2      = (R_ex4-R_ex12)/s2;
[Fs2,F_mu_2]= Hydraulic_force(param,pA2,pB2, s2, dots2,2);
L_e12       = [I_2 Rot_Matrix(phi1)*I_til*pos.bar_hy2_up_1];
L_e4        = [I_2 Rot_Matrix(phi4)*I_til*pos.bar_hy2_up_4];
F2          = Fs2*e_dir2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Global force *********
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qe_1        = L_e1'*F1;          % lift cylinder on body 1
Qe_12       = -L_e12'*F2;        % tilt cylinder reaction on body 1
Qe_4        = L_e4'*F2;          % tilt cylinder on body 4

Q_hyd = [Qe_1+Qe_12;             % body 1: lift boom
         zeros(3,1);             % body 2: tilt boom
         zeros(3,1);             % body 3: bracket 2
         Qe_4];                  % body 4: bracket 1
end