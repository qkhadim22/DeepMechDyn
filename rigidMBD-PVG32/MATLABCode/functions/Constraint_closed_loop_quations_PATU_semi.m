function [C,Cz, Gzdz] = Constraint_closed_loop_quations_PATU_semi(dz_all,R_i,phi_i,dphi_i,Rd,d,joints)
%%%  Calculate the Constraint equation C
I_2 = [1 0;
    0 1];

I_til = [0 -1;
    1 0];
o_2 = [0 0;
    0 0];


%%%% Right hand
R_R = R_i(:,3);
phi_R = phi_i(:,3);
dphi_R = dphi_i(:,3);
A_R = [cos(phi_R) -sin(phi_R);
    sin(phi_R) cos(phi_R)];

bar_up_R = joints(1).bar_ibody;
D_i_R = Di_Matrix_rigid(3,R_i);
e_i_R = ei_Vector_rigid(3,R_i,dphi_i);
L_R = [I_2 A_R*I_til*bar_up_R];
dL_R = [o_2 -dphi_R*A_R*bar_up_R];
T_R = [1 0 0 1 0 0 1 0 0 0 0 0;
    0 1 0 0 1 0 0 1 0 0 0 0;
    0 0 1 0 0 1 0 0 1 0 0 0];

%%%% Left hand
R_L = R_i(:,4);
phi_L = phi_i(:,4);
dphi_L = dphi_i(:,4);
A_L = [cos(phi_L) -sin(phi_L);
    sin(phi_L) cos(phi_L)];

bar_up_L = joints(1).bar_jbody;
D_i_L = Di_Matrix_rigid(4,R_i);
e_i_L = ei_Vector_rigid(4,R_i,dphi_i);
L_L = [I_2 A_L*I_til*bar_up_L];
dL_L = [o_2 -dphi_L*A_L*bar_up_L];
T_L = [1 0 0 0 0 0 0 0 0 1 0 0;
    0 1 0 0 0 0 0 0 0 0 1 0;
    0 0 1 0 0 0 0 0 0 0 0 1];
C = -(R_R+A_R*bar_up_R)+R_L+A_L*bar_up_L;
Cz = -L_R*D_i_R*T_R*Rd + L_L*D_i_L*T_L*Rd;
Gzdz = -dL_R*D_i_R*T_R*Rd*dz_all-L_R*D_i_R*(T_R*d+e_i_R) + dL_L*D_i_L*T_L*Rd*dz_all+L_L*D_i_L*(T_L*d+e_i_L);




