function C = Constraint_quations_postition_PATU_semi(R_i,phi_i,joints)
%%%  Calculate the Constraint equation C

%%%% Right hand
R_R = R_i(:,joints(1).ibody);
phi_R = phi_i(:,joints(1).ibody);
A_R = [cos(phi_R) -sin(phi_R);
    sin(phi_R) cos(phi_R)];

bar_up_R = joints(1).bar_ibody;

%%%% Left hand
R_L = R_i(:,joints(1).jbody);
phi_L = phi_i(:,joints(1).jbody);
A_L = [cos(phi_L) -sin(phi_L);
    sin(phi_L) cos(phi_L)];

bar_up_L = joints(1).bar_jbody;
C = -(R_R+A_R*bar_up_R)+R_L+A_L*bar_up_L;

