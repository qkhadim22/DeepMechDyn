function Cz = Constraint_closed_loop_Jacobian_quations_PATU_semi(R_i,phi_i,rj,joints)
%%%  Calculate the Constraint equation C
I_2 = [1 0;
    0 1];

I_til = [0 -1;
    1 0];
Rd = Rd_function_PATU(rj);

%%%% Right hand
phi_R = phi_i(:,3);
A_R = [cos(phi_R) -sin(phi_R);
    sin(phi_R) cos(phi_R)];

bar_up_R = joints(1).bar_ibody;
D_i_R = Di_Matrix_rigid(3,R_i);

L_R = [I_2 A_R*I_til*bar_up_R]; %Cq1 = [I  A_theta*\bar(u_p)]

T_R = [1 0 0 1 0 0 1 0 0 0 0 0;
    0 1 0 0 1 0 0 1 0 0 0 0;
    0 0 1 0 0 1 0 0 1 0 0 0];

%%%% Left hand
phi_L = phi_i(:,4);
A_L = [cos(phi_L) -sin(phi_L);
    sin(phi_L) cos(phi_L)];

bar_up_L = joints(1).bar_jbody;
D_i_L = Di_Matrix_rigid(4,R_i);
L_L = [I_2 A_L*I_til*bar_up_L];
T_L = [1 0 0 0 0 0 0 0 0 1 0 0;
    0 1 0 0 0 0 0 0 0 0 1 0;
    0 0 1 0 0 0 0 0 0 0 0 1];
Cz = -L_R*D_i_R*T_R*Rd + L_L*D_i_L*T_L*Rd;




