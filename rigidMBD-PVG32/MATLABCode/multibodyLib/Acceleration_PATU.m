function a_out = Acceleration_PATU(q,t,poptim1,poptim2, l1, l2, joints,param,param_bod,pos,bodies)


%%%%%%%%%% Coordinates:Position and Velocity%%%%%%%%%%%%%
   z_i  = q(1:2);
  dz_i  = q(3:4);
[z,dz]  = fun_constraintposition_semi_PATU(t,z_i,dz_i,...
                               joints,param,param_bod); % Relative coordinates
    phi = [z(1), z(1)+z(2), z(1)+z(2)+z(3), z(1)+z(4)]; % Global z 
   dphi = [dz(1), dz(1)+dz(2), dz(1)+dz(2)+dz(3), dz(1)+dz(4)];  % Global dz

[R_i,dRi,rj] = Recursive_position_PATU(phi,dphi,param_bod);   %R_i =g, dRi=dg, rj= r

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Mass matrix, quadratic vector and gravity force ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M_bar  = zeros(12,12);
Qv_bar = zeros(12,1);
Qg_bar = zeros(12,1);

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

Qhy               = Hydralic_force_Patu_CMAES(q,R_i,phi,dRi,dphi,pos,poptim1,poptim2,l1, l2);
Qhy_bar           = D_matrix'*(Qhy);% add mass
    T             = Path_matrix_Patu();
    Rd            = Rd_function_PATU(rj);
    d             = d_vec_function_PATU(dphi,rj);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Constraint equations closed loop ******
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [~,Cz, Gzdz]  = Constraint_closed_loop_quations_PATU_semi(dz,R_i,phi,dphi,Rd,d,joints); % ??????????? check jacobian matrices
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Solve EOM CP ******
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    Czd           = Cz(:,[2  3]);           
    Czi           = Cz(:,[1  4]);   
    Rz            = [1 0;
                    -Czd^(-1)*Czi;
                    0 1]; 
    B             = [0;
                    -Czd^(-1)*Gzdz;
                    0];
    
    SysmL         = Rz.'*Rd.'*(T'*M_bar*T)*Rd*Rz; % First part of Eq. (16)
    SysmR         = Rz.'*Rd.'*T'*(Qv_bar+Qg_bar+Qhy_bar)-Rz.'*Rd.'*T'*M_bar*T*(d+Rd*B); % First part of Eq. (16)
 
a_out             = SysmL\SysmR;
a_out             = a_out(:);


