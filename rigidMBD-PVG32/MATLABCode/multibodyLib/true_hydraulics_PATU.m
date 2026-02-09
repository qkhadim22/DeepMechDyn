function [tspan, x,Z] = true_hydraulics_PATU(data, tEnd,I)
    
    Ts                          = 1e-3;   
    tspan                       = 0:Ts:tEnd;                   % Time period
    N                           = length(tspan);             % Number of steps
   [param]                       = Get_parameters_Patu();
[bodies,joints,pos,param_bod]   = Get_bodies_PATU_model_semi();

    p1                          =     6.608017392310942e+06;%data(5,1);
    p2                          = data(6,1);
    p3                          =  5.224301361479872e+05;%data(7,1);      % Pressure P2,4.780503480752360e+06
    p4                          = data(8,1);      % Pressure P3,7.477028535636445e+06
     x                          = zeros(8,N);
     U1                         = data(11,1:N); % 0*data(11,1:N)+5.8*ones(1,N)
     U2                         = data(12,1:N); %0*data(12,1:N)+
     p_P                        = data(13,1:N);
     p_t                        = data(14,1:N);
     Z                          = zeros(4,N);
     Z(:,1)                     = [data(2,1);data(3,1);0;0];

  % y                             = [data(12,1:N);data(13,1:N);data(14,1:N);data(15,1:N);data(6,1:N);data(7,1:N);data(8,1:N);data(9,1:N);data(2,1:N);data(3,1:N);U1; U2; ];
   % y                             = [data(12,1:N);data(13,1:N);data(14,1:N);data(15,1:N);data(6,1:N);data(7,1:N);data(8,1:N);data(9,1:N);data(2,1:N);data(3,1:N);U1; U2; ];

  x(:,1)                        = [deg2rad(data(1,1));deg2rad(data(2,1))-deg2rad(data(1,1));0;0;
                                     p1;p2;p3;p4];

for k = 2:N

 [x(:, k), Z(:, k)]     = Rk4_Patu(@eom, Ts,tspan(k),x(:, k-1), U1(:, k),U2(:, k),p_P(k),p_t(k), I);

end


function [dy, Z] = eom(t,x, U1, U2, p_P,p_t, I)
 t
z_i             = x(1:2);
dz_i            = x(3:4);
p_hyd           = x(5:8);
tau             = 30e-3;

[z,dz] = fun_constraintposition_semi_PATU(t,z_i,dz_i,joints,param,param_bod); % Relative coordinates


phi = [z(1), z(1)+z(2), z(1)+z(2)+z(3), z(1)+z(4)]; % Global z 
dphi = [dz(1), dz(1)+dz(2), dz(1)+dz(2)+dz(3), dz(1)+dz(4)];  % Global dz

[R_i,dRi,rj] = Recursive_position_PATU(phi,dphi,param_bod);   %R_i =g, dRi=dg, rj= r

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Mass matrix, quadratic vector and gravity force ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M_bar = zeros(12,12);
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Hydralic PATU ******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 [Qhy, dp_hyd,s1, dots1, s2, dots2] = Hydralic_force_Patu_rigid(t, R_i,phi,dRi,dphi,p_hyd,param,U1,U2,pos,p_P,p_t,I);

% [Qhy, dp_hyd] = Hydralic_force_Patu_new_rigid(t, R_i,phi,dRi,dphi,p_hyd,param,U1,U2,pos,p_P,p_t);
% Q_wei = External_weight(phi,param,pos); % add mass

Qhy_bar       = D_matrix'*(Qhy);% add mass
    T             = Path_matrix_Patu();
    Rd            = Rd_function_PATU(rj);
    d             = d_vec_function_PATU(dphi,rj);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Constraint equations closed loop ******
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [C,Cz, Gzdz] = Constraint_closed_loop_quations_PATU_semi(dz,R_i,phi,dphi,Rd,d,joints); % ??????????? check jacobian matrices
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Solve EOM CP ******
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    Czd = Cz(:,[2  3]);           
    Czi = Cz(:,[1  4]);   
    Rz = [1 0;
         -Czd^(-1)*Czi;
         0 1]; 
    B = [0;
         -Czd^(-1)*Gzdz;
         0];
    
    SysmL = Rz.'*Rd.'*(T'*M_bar*T)*Rd*Rz; % First part of Eq. (16)
    SysmR = Rz.'*Rd.'*T'*(Qv_bar+Qg_bar+Qhy_bar)-Rz.'*Rd.'*T'*M_bar*T*(d+Rd*B); % First part of Eq. (16)
 
ddz_i = SysmL\SysmR;

dy(1:2,1) = dz_i;
dy(3:4,1) = ddz_i;
dy(5:8,1) = dp_hyd;
dy        = dy(:);
Z(1)      = s1;
Z(2)      = s2;
Z(3)      = dots1;
Z(4)      = dots2;
end
end