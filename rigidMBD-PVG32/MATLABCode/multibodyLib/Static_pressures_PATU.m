function [x0] = Static_pressures_PATU(z2_0, data, param, loadCase)

% Piston-side pressures ph1 (lift) and ph3 (tilt) that put the model in static
% equilibrium at the pose of data(:,1), using the measured rod-side pressures
% ph2 = data(6,1) and ph4 = data(8,1).

[bodies,joints,pos,param_bod] = Get_bodies_PATU_model_semi(loadCase);

z_i    = [deg2rad(data(1,1)); z2_0];

[z,dz] = fun_constraintposition_semi_PATU(0,z_i,[0;0],joints,param,param_bod);
phi    = (param.Tz*z).';
dphi   = zeros(1,param.nz);
[R_i,dRi,rj] = Recursive_position_PATU(phi,dphi,param_bod);

Qg_bar   = zeros(3*param.nb,1);
D_matrix = zeros(3*param.nb,3*param.nb);
for i = 1:length(bodies)
    D_i = Di_Matrix_rigid(i,R_i);
    Qg_bar(3*(i-1)+1:3*i,1)               = D_i'*[0 -bodies(i).m*param.g 0]';
    D_matrix(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = D_i;
end

T        = Path_matrix_Patu(param);
Rd       = Rd_function_PATU(rj);
d        = d_vec_function_PATU(dphi,rj,param);
[~,Cz,~] = Constraint_closed_loop_quations_PATU_semi(dz,R_i,phi,dphi,Rd,d,joints);
Rz       = zeros(param.nz,numel(param.Index_ind));
Rz(param.Index_ind,:) = eye(numel(param.Index_ind));
Rz(param.Index_dep,:) = -Cz(:,param.Index_dep)\Cz(:,param.Index_ind);
P        = Rz.'*Rd.'*T';
% [Qhy,s1, dots1, s2, dots2, F_mu_1, F_mu_2, F_h_1,F_h_2] = Global_force_hydraulic(R_i,phi,dRi,dphi,[pA(1); data(6,1); pA(2); data(8,1)], pos,param);


% generalized force residual for given piston-side pressures (linear in them)
res = @(pA) P*(Qg_bar + D_matrix'*Global_force_hydraulic(R_i,phi,dRi,dphi, ...
    [pA(1); data(6,1); pA(2); data(8,1)], pos, param));
r0  = res([0;0]);
J   = [res([1e5;0])-r0, res([0;1e5])-r0] / 1e5;
p   = -J\r0;


% initial state: [z1; z4; dz1; dz4; ph1; ph2; ph3; ph4; u1; u2]
x0 = [z_i; 0; 0; p(1); data(6,1); p(2); data(8,1); data(11,1); data(12,1)];
end