function [simData] = true_hydraulics_PATU(data, tEnd,param, loadCase)
    
    Ts                              = 1e-3;   
    tspan                           = 0:Ts:tEnd;                  
    N                               = length(tspan);             

   [bodies,joints,pos,param_bod]    = Get_bodies_PATU_model_semi(loadCase);

    % States and cylinders data
    x                               = zeros(10,N);
    Z                               = zeros(14,N);
 
    % Initial pose from measured s2: z2 from the cylinder triangle, z4 by attaching bracket 2 to the tilt boom
    s2_meas                         = data(10,1);
    z1_meas                         = deg2rad(data(1,1));
    z2_meas                         = deg2rad(data(2,1)) - z1_meas;

    [param.z_dep0, param.z2_0]        = Initial_pose_from_s2_PATU(s2_meas,z1_meas,z2_meas,  joints, pos, param, param_bod);
    z                           = fun_constraintposition_semi_PATU(0, [z1_meas; param.z2_0], [0;0], joints, param, param_bod);

   x(:,1)                       = Static_pressures_PATU(param.z2_0, data, param, loadCase);

        Z(:,1)         = [data(2,1);data(3,1);0;0;0;0;0;0;0;0;0;0;0;0];
        U1             = data(11,1:N); 
        U2             = data(12,1:N); 
        p_P            = data(13,1:N);

z_dep_prev = param.z_dep0;         % warm start for the position problem (updated in eom)
T          = Path_matrix_Patu(param);  % constant path matrix, built once

for k = 2:N

 dU1                 = (U1(:, k-1)-x(9, k-1))/param.tau;
 dU2                 = (U2(:, k-1)-x(10, k-1))/param.tau;
 Uk1                 =   U1(:, k-1)+Ts*dU1;
 Uk2                 =  U2(:, k-1)+Ts*dU2;

 optionalVec         = [Uk1; Uk2;p_P(k);param.p_t;dU1;dU2];

 [x(:, k), Z(:, k)]   = Rk4_Patu(@eom, Ts,tspan(k),x(:, k-1));

end



function [dy, Z] = eom(t,x)
% t
z_i             = x(1:2);
dz_i            = x(3:4);
p_hyd           = x(5:8);

[z,dz] = fun_constraintposition_semi_PATU(t,z_i,dz_i,joints,param,param_bod,z_dep_prev); % Relative coordinates
z_dep_prev = z(param.Index_dep);
phi = (param.Tz*z).'; % Global z 
dphi = (param.Tz*dz).';  % Global dz

[R_i,dRi,rj] = Recursive_position_PATU(phi,dphi,param_bod);   %R_i =g, dRi=dg, rj= r

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Mass matrix, quadratic vector and gravity force ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M_bar = zeros(3*param.nb,3*param.nb);
Qv_bar = zeros(3*param.nb,1);
Qg_bar = zeros(3*param.nb,1);
D_matrix = zeros(3*param.nb,3*param.nb);

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
 [Qhy,s1, dots1, s2, dots2, F_mu_1, F_mu_2, F_h_1,F_h_2] = Global_force_hydraulic(R_i,phi,dRi,dphi,p_hyd, pos,param);

% model = param.BulkModulus(I);
[pA1_dot,pB1_dot,Be_h1,Be_h2, u1]       = pressure_derivatives(t,param, s1,dots1,p_hyd(1),p_hyd(2),optionalVec(1), 1);
[pA2_dot,pB2_dot,Be_h3,Be_h4, u2]       = pressure_derivatives(t,param, s2,dots2,p_hyd(3),p_hyd(4), optionalVec(2), 2);


Qhy_bar                                 = D_matrix'*(Qhy);% add mass
% T                                       = Path_matrix_Patu();
Rd                                      = Rd_function_PATU(rj);
d                                       = d_vec_function_PATU(dphi,rj,param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Constraint equations closed loop ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[C,Cz, Gzdz] = Constraint_closed_loop_quations_PATU_semi(dz,R_i,phi,dphi,Rd,d,joints); % ??????????? check jacobian matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Solve EOM CP ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%
Czd = Cz(:,param.Index_dep);           
Czi = Cz(:,param.Index_ind);   
Rz = zeros(param.nz,numel(param.Index_ind));
Rz(param.Index_ind,:) = eye(numel(param.Index_ind));
Rz(param.Index_dep,:) = -Czd\Czi;
B = zeros(param.nz,1);
B(param.Index_dep) = -Czd\Gzdz;
    
SysmL = Rz.'*Rd.'*(T'*M_bar*T)*Rd*Rz; % First part of Eq. (16)
SysmR = Rz.'*Rd.'*T'*(Qv_bar+Qg_bar+Qhy_bar)-Rz.'*Rd.'*T'*M_bar*T*(d+Rd*B); % First part of Eq. (16)
 
ddz_i = SysmL\SysmR;

dy(1:2,1) = dz_i;
dy(3:4,1) = ddz_i;
dy(5,1)   = pA1_dot;
dy(6,1)   = pB1_dot;
dy(7,1)   = pA2_dot;
dy(8,1)   = pB2_dot;
dy(9:10,1)= optionalVec(5:6);
dy        = dy(:);
Z(1)      = 1000*s1;
Z(2)      = 1000*s2;
Z(3)      = 1000*dots1;
Z(4)      = 1000*dots2;
Z(5)      = F_mu_1;
Z(6)      = F_mu_2;
Z(7)      = F_h_1;
Z(8)      = F_h_2;
Z(9)      = Be_h1;
Z(10)     = Be_h2;
Z(11)     = Be_h3;
Z(12)     = Be_h4;
Z(13)     = u1;
Z(14)     = u2;
end

simData.Time         = tspan;
simData.states       = x;
simData.Hycyls       = Z;
simData.states(2,:) = x(2,:) + (z2_meas - param.z2_0);     % shift to inclinometer reference

end