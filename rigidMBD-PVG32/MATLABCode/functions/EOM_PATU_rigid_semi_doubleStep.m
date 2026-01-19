function dy = EOM_PATU_rigid_semi_doubleStep(t,y,param,param_bod,bodies,joints,pos,U1,U2,pP)
t
z_i = y(1:param.tot_dof);
dz_i = y(param.tot_dof + 1:2* param.tot_dof);
p_hyd = y(2* param.tot_dof+1:end);


[z,dz] = fun_constraintposition_semi_PATU(t,z_i,dz_i,joints,param,param_bod); % Relative coordinates


phi = [z(1), z(1)+z(2), z(1)+z(2)+z(3), z(1)+z(4)]; % Global z 
dphi = [dz(1), dz(1)+dz(2), dz(1)+dz(2)+dz(3), dz(1)+dz(4)];  % Global dz

[R_i,dRi,rj] = Recursive_position_PATU(phi,dphi,param_bod);   %R_i =g, dRi=dg, rj= r

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Mass matrix, quadratic vector and gravity force ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M_bar = zeros(param.ndof_full,param.ndof_full);
Qv_bar = zeros(param.ndof_full,1);
Qg_bar = zeros(param.ndof_full,1);

for i = 1:length( bodies)
    D_i = Di_Matrix_rigid(i,R_i);
    e_i = ei_Vector_rigid(i,R_i,dphi);
    Mi = [bodies(i).m 0 0;
        0 bodies(i).m 0;
        0 0 bodies(i).J]; 
    Qvi = zeros(3,1);
    Qgi = [0 -bodies(i).m*param.g 0]';
    
    M_bar(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = D_i'* Mi*D_i; %Equation 92 in Dopaico paper
    Qv_bar(3*(i-1)+1:3*i,1) = D_i'*(Qvi-Mi*e_i);       %Equation 93 in Dopaico paper
    Qg_bar(3*(i-1)+1:3*i,1) = D_i'*Qgi;               
    D_matrix(3*(i-1)+1:3*i,3*(i-1)+1:3*i)=D_i; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Hydralic PATU ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Qhy, dp_hyd] = Hydralic_force_Patu_rigid(R_i,phi,dRi,dphi,p_hyd,param,U1,U2,pos,pP);

% Q_wei = External_weight(phi,param,pos); % add mass

Qhy_bar = D_matrix'*(Qhy);% add mass
% Qhy_bar = D_matrix'*Qhy;
T = Path_matrix_Patu();
Rd = Rd_function_PATU(rj);
d = d_vec_function_PATU(dphi,rj);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Constraint equations closed loop ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[C,Cz, Gzdz] = Constraint_closed_loop_quations_PATU_semi(dz,R_i,phi,dphi,Rd,d,joints); % ??????????? check jacobian matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Solve EOM CP ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%
Czd = Cz(:,param.Index_dep);           
Czi = Cz(:,param.Index_ind);   
Rz = [1 0;
     -Czd^(-1)*Czi;
     0 1]; 
B = [0;
     -Czd^(-1)*Gzdz;
     0];

SysmL = Rz.'*Rd.'*(T'*M_bar*T)*Rd*Rz; % First part of Eq. (16)
SysmR = Rz.'*Rd.'*T'*(Qv_bar+Qg_bar+Qhy_bar)-Rz.'*Rd.'*T'*M_bar*T*(d+Rd*B); % First part of Eq. (16)
 
ddz_i = SysmL\SysmR;

dy(1:param.tot_dof,1) = dz_i;
dy(param.tot_dof+1:2*param.tot_dof,1) = ddz_i;
dy(2*param.tot_dof+1:length(y),1) = dp_hyd;

