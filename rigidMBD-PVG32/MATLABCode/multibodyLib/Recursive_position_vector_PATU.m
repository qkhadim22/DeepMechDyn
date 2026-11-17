function [R_i,rj] = Recursive_position_vector_PATU(phi,param_bod)
% New numbering: 1 lift boom, 2 tilt boom, 3 bracket 2, 4 bracket 1
% Tree: 1 -> 2 -> 3,  1 -> 4

phi1            = phi(1);
phi2            = phi(2);
phi3            = phi(3);
phi4            = phi(4);

A               = @(theta) [cos(theta)   -sin(theta); sin(theta)   cos(theta)];

% Joint locations (global)
rj(:,1)         = [0; 0];                                                                  % J1: lift boom base
rj(:,2)         = A(phi1)*[param_bod.u1_bar_j1_j2x; param_bod.u1_bar_j1_j2y];              % J2: tilt pivot
rj(:,3)         = rj(:,2) + A(phi2)*[param_bod.u2_bar_j2_j3x; param_bod.u2_bar_j2_j3y];    % J3: lug C on tilt boom
rj(:,4)         = A(phi1)*[param_bod.u1_bar_j1_j4x; param_bod.u1_bar_j1_j4y];              % J4: bracket-1 pivot

% Centre-of-mass positions (global)
R_i(:,1)        = rj(:,1) + A(phi1)*[param_bod.u1_bar_j1_o1x; param_bod.u1_bar_j1_o1y];
R_i(:,2)        = rj(:,2) + A(phi2)*[param_bod.u2_bar_j2_o2x; param_bod.u2_bar_j2_o2y];
R_i(:,3)        = rj(:,3) + A(phi3)*[param_bod.u3_bar_j3_o3x; param_bod.u3_bar_j3_o3y];
R_i(:,4)        = rj(:,4) + A(phi4)*[param_bod.u4_bar_j4_o4x; param_bod.u4_bar_j4_o4y];
end