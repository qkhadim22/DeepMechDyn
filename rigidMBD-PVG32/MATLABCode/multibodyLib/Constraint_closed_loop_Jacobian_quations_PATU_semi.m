function Cz = Constraint_closed_loop_Jacobian_quations_PATU_semi(R_i,phi_i,rj,joints)
%%%  Calculate the Constraint equation C
I_2 = [1 0;
    0 1];

I_til = [0 -1;
    1 0];
Rd = Rd_function_PATU(rj);

%%%% Right hand
phi_R = phi_i(:,joints(1).ibody);
A_R = [cos(phi_R) -sin(phi_R);
    sin(phi_R) cos(phi_R)];

bar_up_R = joints(1).bar_ibody;
D_i_R = Di_Matrix_rigid(joints(1).ibody,R_i);

L_R = [I_2 A_R*I_til*bar_up_R]; %Cq1 = [I  A_theta*\bar(u_p)]

T_R = joints(1).T_i;

%%%% Left hand  
phi_L = phi_i(:,joints(1).jbody);
A_L = [cos(phi_L) -sin(phi_L);
    sin(phi_L) cos(phi_L)];

bar_up_L = joints(1).bar_jbody;
D_i_L = Di_Matrix_rigid(joints(1).jbody,R_i);
L_L = [I_2 A_L*I_til*bar_up_L];
T_L = joints(1).T_j;
Cz = -L_R*D_i_R*T_R*Rd + L_L*D_i_L*T_L*Rd;




