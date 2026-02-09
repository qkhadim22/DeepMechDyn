function [R_i,rj] = Recursive_position_vector_PATU(phi,param_bod)
 
phi1 = phi(1);
phi2 = phi(2);
phi3 = phi(3);
phi4 = phi(4);


R_i(:,1) = [param_bod.u1_bar_j1_o1x*cos(phi1) - param_bod.u1_bar_j1_o1y*sin(phi1);
            param_bod.u1_bar_j1_o1y*cos(phi1) + param_bod.u1_bar_j1_o1x*sin(phi1)];
R_i(:,2) = [param_bod.u1_bar_j1_j2x*cos(phi1) + param_bod.u2_bar_j2_o2x*cos(phi2) - param_bod.u1_bar_j1_j2y*sin(phi1) - param_bod.u2_bar_j2_o2y*sin(phi2);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u2_bar_j2_o2y*cos(phi2) + param_bod.u1_bar_j1_j2x*sin(phi1) + param_bod.u2_bar_j2_o2x*sin(phi2)];
R_i(:,3) = [param_bod.u1_bar_j1_j2x*cos(phi1) + param_bod.u2_bar_j2_j3x*cos(phi2) + param_bod.u3_bar_j3_o3x*cos(phi3) - param_bod.u1_bar_j1_j2y*sin(phi1) - param_bod.u2_bar_j2_j3y*sin(phi2) - param_bod.u3_bar_j3_o3y*sin(phi3);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u2_bar_j2_j3y*cos(phi2) + param_bod.u3_bar_j3_o3y*cos(phi3) + param_bod.u1_bar_j1_j2x*sin(phi1) + param_bod.u2_bar_j2_j3x*sin(phi2) + param_bod.u3_bar_j3_o3x*sin(phi3)];       
R_i(:,4) = [param_bod.u1_bar_j1_j4x*cos(phi1) + param_bod.u4_bar_j4_o4x*cos(phi4) - param_bod.u1_bar_j1_j4y*sin(phi1) - param_bod.u4_bar_j4_o4y*sin(phi4);
            param_bod.u1_bar_j1_j4y*cos(phi1) + param_bod.u4_bar_j4_o4y*cos(phi4) + param_bod.u1_bar_j1_j4x*sin(phi1) + param_bod.u4_bar_j4_o4x*sin(phi4)];

rj(:,1) = [ 0;
            0];
rj(:,2) = [ param_bod.u1_bar_j1_j2x*cos(phi1) - param_bod.u1_bar_j1_j2y*sin(phi1);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u1_bar_j1_j2x*sin(phi1)];
rj(:,3) = [ param_bod.u1_bar_j1_j2x*cos(phi1) + param_bod.u2_bar_j2_j3x*cos(phi2) - param_bod.u1_bar_j1_j2y*sin(phi1) - param_bod.u2_bar_j2_j3y*sin(phi2);
            param_bod.u1_bar_j1_j2y*cos(phi1) + param_bod.u2_bar_j2_j3y*cos(phi2) + param_bod.u1_bar_j1_j2x*sin(phi1) + param_bod.u2_bar_j2_j3x*sin(phi2)];
rj(:,4) = [ param_bod.u1_bar_j1_j4x*cos(phi1) - param_bod.u1_bar_j1_j4y*sin(phi1);
            param_bod.u1_bar_j1_j4y*cos(phi1) + param_bod.u1_bar_j1_j4x*sin(phi1)];

      