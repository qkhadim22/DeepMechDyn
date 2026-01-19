function [C] = fun_constraint_quations_Revolute_semi_PATU(z_dep,z_ind,joints,param,param_bod)
%%%  Calculate the Constraint equation C
z( param.Index_ind,1) = z_ind;
z( param.Index_dep,1) = z_dep;

phi1 = z(1);
phi2 = z(1)+z(2);
phi3 = z(1)+z(2)+z(3);
phi4 = z(1)+z(4);
phi_i = [phi1 phi2 phi3 phi4];
R_i(:,1) = [param_bod.u1_bar_j1_o1x*cos(phi1) - param_bod.u1_bar_j1_o1y*sin(phi1);
            param_bod.u1_bar_j1_o1y*cos(phi1) + param_bod.u1_bar_j1_o1x*sin(phi1)];
R_i(:,2) = [param_bod.u1_bar_j1_j2x*cos(phi1) + param_bod.u2_bar_j2_o2x*cos(phi2) - param_bod.u1_bar_j1_j2y*sin(phi1) - param_bod.u2_bar_j2_o2y*sin(phi2);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u2_bar_j2_o2y*cos(phi2) + param_bod.u1_bar_j1_j2x*sin(phi1) + param_bod.u2_bar_j2_o2x*sin(phi2)];
R_i(:,3) = [param_bod.u1_bar_j1_j2x*cos(phi1) + param_bod.u2_bar_j2_j3x*cos(phi2) + param_bod.u3_bar_j3_o3x*cos(phi3) - param_bod.u1_bar_j1_j2y*sin(phi1) - param_bod.u2_bar_j2_j3y*sin(phi2) - param_bod.u3_bar_j3_o3y*sin(phi3);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u2_bar_j2_j3y*cos(phi2) + param_bod.u3_bar_j3_o3y*cos(phi3) + param_bod.u1_bar_j1_j2x*sin(phi1) + param_bod.u2_bar_j2_j3x*sin(phi2) + param_bod.u3_bar_j3_o3x*sin(phi3)];       
R_i(:,4) = [param_bod.u1_bar_j1_j4x*cos(phi1) + param_bod.u4_bar_j4_o4x*cos(phi4) - param_bod.u1_bar_j1_j4y*sin(phi1) - param_bod.u4_bar_j4_o4y*sin(phi4);
            param_bod.u1_bar_j1_j4y*cos(phi1) + param_bod.u4_bar_j4_o4y*cos(phi4) + param_bod.u1_bar_j1_j4x*sin(phi1) + param_bod.u4_bar_j4_o4x*sin(phi4)];




nj = length(joints);   % number of joints

for i = 1:nj
    ib = joints(i).ibody;
    jb = joints(i).jbody;
    m = 2*i-1:2*i;

    %%%% Right hand
    R_R = R_i(:,ib);
    phi_R = phi_i(:,ib);
    A_R = [cos(phi_R) -sin(phi_R);
        sin(phi_R) cos(phi_R)];
    
    bar_up_R = joints(i).bar_ibody;
    %%%% Left hand
    R_L = R_i(:,jb);
    phi_L = phi_i(:,jb);
    A_L = [cos(phi_L) -sin(phi_L);
           sin(phi_L) cos(phi_L)];
    
    bar_up_L = joints(i).bar_jbody;
    C(m,1) = -(R_R+A_R*bar_up_R)+R_L+A_L*bar_up_L; 
end


