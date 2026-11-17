function [phi_dep0, z2_0] = Initial_pose_from_s2_PATU(s2, z1, z2, joints, pos, param, param_bod)

z           =  [0; 0; 0; 0];
Arot        = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
wp          = @(theta) atan2(sin(theta), cos(theta));
[R0, rj0]   = Recursive_position_vector_PATU(z, param_bod);

%% Calculation of z4
A_pt        = R0(:,1) + pos.bar_hy2_up_1;                                            % tilt cylinder base
D_loc       = [param_bod.u4_bar_j4_o4x; param_bod.u4_bar_j4_o4y] + pos.bar_hy2_up_4;   % J4 -> cylinder pin (bracket-1 frame)
J4_pt       = rj0(:,4);
ab          = A_pt - J4_pt;
cosB        = (norm(ab)^2 + norm(D_loc)^2 - s2^2) / (2*norm(ab)*norm(D_loc));

if abs(cosB) > 1
    warning('Triangle J4-A-D cannot close for s2 = %.4f (cosB = %.3f)', s2, cosB);
end

z4_0        = wp( atan2(ab(2),ab(1)) - acos(max(-1,min(1,cosB))) - atan2(D_loc(2),D_loc(1)) );

%B_pt        = J4_pt + Arot(z4_0)*D_loc; 
%plot_triangle_PATU(A_pt, B_pt, J4_pt, z(1), z4_0, param.cyl(2).l_cyl)

%% Calculation of z2 and z3 (z1 and z4_0 fixed)
P_loc       = [param_bod.u4_bar_j4_o4x; param_bod.u4_bar_j4_o4y] + joints(1).bar_jbody;
e3          = [param_bod.u3_bar_j3_o3x; param_bod.u3_bar_j3_o3y] + joints(1).bar_ibody;  % C  -> loop pin (bracket 2)
c           = [param_bod.u2_bar_j2_j3x; param_bod.u2_bar_j2_j3y];                       % J2 -> C (tilt boom)

L3          = norm(e3);                            % bracket 2 length (0.48 m)
r           = norm(c);                             % J2 -> C   (tilt-boom lug)
theta       = atan2(c(2), c(1));                   % lug J2->C relative to tilt-boom x-axis (constant)

% fixed points (do not depend on z2)
[~, rjC]    = Recursive_position_vector_PATU([z1; z1+z2; 0; z1+z4_0], param_bod);       % global frame
J2_pt       = rjC(:,2);                            % tilt pivot
J4_pt       = rjC(:,4);                            % bracket-1 pivot
pin         = J4_pt + Arot(z1+z4_0)*P_loc;         % loop pin from s2-based z4

% z2 from the loop: triangle J2 - C - pin (sides r, L3, dJ2P)
dJ2P        = norm(pin - J2_pt);                                                  % J2 -> pin
theta2      = acos((r^2 + dJ2P^2 - L3^2) / (2*r*dJ2P));                           % angle at J2 between J2->pin and J2->C
z2_cand     = atan2(pin(2)-J2_pt(2), pin(1)-J2_pt(1)) + [-theta2, theta2] - theta - z1;   % two possible tilt angles
[~, k]      = min(abs(wp(z2_cand - z2)));                                         % closest to inclinometer
z2_0        = z2_cand(k);                                                         % tilt boom (relative)

% lug C with the new z2
[~, rjC]    = Recursive_position_vector_PATU([z1; z1+z2_0; 0; z1+z4_0], param_bod);     % global frame
J3_pt       = rjC(:,3);                            % lug C

% angle at C between lug (C->J2) and bracket 2 (C->pin), signed
gamma       = atan2(J2_pt(2)-J3_pt(2), J2_pt(1)-J3_pt(1)) - atan2(pin(2)-J3_pt(2), pin(1)-J3_pt(1));
z3_0        = wp(theta - gamma);
dCP         = norm(pin - J3_pt);                   % check: should equal L3

fprintf('z2: loop %.2f deg, inclinometer %.2f deg | C-pin = %.4f m, L3 = %.4f m\n', ...
    rad2deg(z2_0), rad2deg(z2), dCP, L3);

phi_dep0 = [z3_0; z4_0];
%plot_PATU_config([z1; z2_0; z3_0; z4_0], param, param_bod, joints, s2);

end