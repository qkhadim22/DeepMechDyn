function [param] = Get_parameters_Patu(BeCase)
if nargin < 1, BeCase = "Standard"; end          % "Standard" or "Identified" (Table II)
%% Degrees of freedom
param.g             = 9.8066;
param.ndof          = 1;
param.nqdof         = 3;
param.tot_dof       = 2;
param.Index_ind     = [1 2];     % z1 lift boom, z2 tilt boom (independent)
param.Index_dep     = [3 4];     % z3 bracket 2, z4 bracket 1 (dependent)

% bodies: 1 lift boom, 2 tilt boom (+ext+payload), 3 bracket 2, 4 bracket 1
param.nb            = 4;                          % number of bodies
param.nz            = 4;                          % number of relative joint coordinates
param.ncyl          = 2;                          % number of hydraulic cylinders
param.parent        = [0 1 2 1];                  % 1<-ground, 2<-1 (tilt pivot), 3<-2 (lug C), 4<-1 (bracket-1 pivot)
param.Tz            = [1 0 0 0;                   % path matrix: global angles phi = Tz*z
                       1 1 0 0;                   % phi2 = z1+z2        (tilt boom)
                       1 1 1 0;                   % phi3 = z1+z2+z3     (bracket 2)
                       1 0 0 1];                  % phi4 = z1+z4        (bracket 1)
%param.z_dep0        = [2.5862;-2.5501];           % was [2.5862;-2.5501]
% param.z_dep0        = [2.3862;-2.8501];           % was [2.5862;-2.5501]

%% Hydraulic parameters
param.VA            =  pi/4*(12.7e-3)^2*1.5;                 % Volume of line between valve and piston side
param.VB            =  pi/4*(12.7e-3)^2*1.5;                 % Volume of line between valve and piston-rod side
param.BeO           =  1600e6;                                % Oil bulk modulus          (Table II: beta_o)
param.Beh           =  700e6;                                 %                           (Table II: beta_h)
param.Bec           =  2.1e11;                                %                           (Table II: beta_c)
param.p_t           =  1e5;                                   % Tank presssure
param.tau           = 30e-3;


param.betaMin       = 2;
% param.betaMaxA      = 1000e6;                               % set in the Table II block below
% param.betaMaxB      = 700e4; %700e4                         % set in the Table II block below
% param.pBetaA        = 30e5; %Full beta (96%) at 60 bar, 70e5  % set in the Table II block below
% param.pBetaB        = 30e5; %Full beta (96%) at 60 bar,70e5   % set in the Table II block below


%PVG32 valve parameters
param.valve(1).u0   = (0.8)/7;%0.8/7
param.valve(1).p_ref= 7e5;
param.valve(1).cl   = 1e-10;
param.valve(1).U_N  = 5.8;
param.valve(1).Q0   = 35/6e4;
param.valve(1).Q1   = 40/6e4;
param.valve(1).Kv0  = (param.valve(1).Q0) / sqrt(param.valve(1).p_ref);
param.valve(1).Kv1  = (param.valve(1).Q1) / sqrt(param.valve(1).p_ref);

param.valve(2).u0   = (0.8)/7;%0.8/7
param.valve(2).p_ref= 7e5;
param.valve(2).cl   = 1e-12;
param.valve(2).U_N  = 5.80;
param.valve(2).Q0   = 40/6e4;
param.valve(2).Kv0  = (param.valve(2).Q0) / sqrt(param.valve(2).p_ref);
Q1                  = 40/6e4;
param.Kv            = (Q1) / sqrt(param.valve(2).p_ref);



% Cylinder 1 parameters
param.cyl(1).A_cyl  = (pi/4)*(100e-3)^2;                          % bore 100, drawing 100/56       
param.cyl(1).A_rod  = param.cyl(1).A_cyl-(pi/4)*(56e-3)^2;       % rod 56                                         
param.cyl(1).l_rod  = 535e-3;                                    % Rod length, stroke 535
param.cyl(1).l_cyl  = 820e-3;                                    % Cylinder length, min length: drawing 793–800, your data fits 820 
param.cyl(1).Fc     =  3.5*1260;  
param.cyl(1).Fs     =  3.5*1400; 
param.cyl(1).sig2   =  3.5*1000;  
param.cyl(1).vs     =  5e-3;  
param.cyl(1).ld     = 40e-3;
param.cyl(1).k      = 1e5;
param.cyl(1).c      = 5e4;
param.cyl(1).V_air1 = 1/100;
param.cyl(1).V_air2 = 38/100;
% param.cyl(1).alphaA = 2/100;                                   % set in the Table II block below
% param.cyl(1).alphaB = 38/100;                                  % set in the Table II block below
% param.cyl(1).alphaA = 1/100;
% param.cyl(1).alphaB = 38/100;

%% Effective bulk modulus parameters (Table II)
% p_a = 1 bar and k = 1.4 are set inside Wylie, Nykanen and Ruan (P0, lambda)
param.BeCase = BeCase;
switch BeCase
    case "Standard"
        param.betaMaxA      = 1000e6;   % beta_eh1,max
        param.betaMaxB      = 1000e6;   % beta_eh2,max
        param.pBetaA        = 30e5;     % p_beta_eh1
        param.pBetaB        = 30e5;     % p_beta_eh2
        param.cyl(1).alphaA = 1/100;    % alpha_h1
        param.cyl(1).alphaB = 1/100;    % alpha_h2
        param.pCA           = 80e5;     % p_Ch1
        param.pCB           = 80e5;     % p_Ch2
    case "Identified"
        param.betaMaxA      = 1600e6;   % beta_eh1,max
        param.betaMaxB      = 300e6;    % beta_eh2,max
        param.pBetaA        = 80e5;     % p_beta_eh1
        param.pBetaB        = 50e5;     % p_beta_eh2
        param.cyl(1).alphaA = 1/100;    % alpha_h1
        param.cyl(1).alphaB = 38/100;   % alpha_h2
        param.pCA           = 80e5;     % p_Ch1
        param.pCB           = 50e5;     % p_Ch2
    otherwise
        error('Unknown BeCase: use "Standard" or "Identified"');
end

param.cyl(1).Kp     =  0;     
param.cyl(2).Kp     =  0;


% Cylinder 1 parameters
param.cyl(2).A_cyl  = (pi/4)*(100e-3)^2;                         % Piston side area        
param.cyl(2).A_rod  = param.cyl(2).A_cyl-(pi/4)*(56e-3)^2;       % Piston side area                                         
param.cyl(2).l_rod  = 780e-3;                                    % Rod length
param.cyl(2).l_cyl  = 1050e-3;                                    % Cylinder length
param.cyl(2).Fc     =  210*3;  
param.cyl(2).Fs     =  350*3; 
param.cyl(2).sig2   =  330*3;  
param.cyl(2).vs     =  5.0e-4;  
param.cyl(2).ld     = 200e-3;
param.cyl(2).k      = 10e6;
param.cyl(2).c      = 5e3;
param.cyl(2).V_air1 =2e-6;
param.cyl(2).V_air2 = 2e-6;
param.cyl(2).alphaA = 2e-7;
param.cyl(2).alphaB = 2e-8;
param.p_P           = 92e5;
% Friction parameters

%% Valve parameters

% param.valve_models  = ["PVG32_old"   "PVG32_flow_basic" "PVG32_flow_assymetricDB"];
% param.BulkModulus       = ["Nominal" "Nykanen" "Ruan" "Wylie" "Pressures"];

param.valve_models  = ["PVG32_flow_assymetricDB"];
param.BulkModulus       = [  "Wylie" ];
% param.BulkModulus       = ["Pressures" ];
%param.valve_models  = ["PVG32_old"];


param.V_nom         = 7.5;
param.u_dl          = 5.20;      % 5.4
param.u_dh          = 6.2;       % 6.6

param.Q_nomA        = 62/60000;  % l/min
param.Q_nomB        = 50/60000;  % l/min

Q_A                 = 95/60000;
p_A                 = 53.5e5;

Q_B                 = 145/60000;
p_B                 = 35e5;

param.Dp            = 7e5;
param.p_ref         = 20e5;

param.Cv_PtoA       = param.Q_nomA / ((4.4 - 2.5)       * sqrt(param.p_ref));
param.Cv_PtoB       = param.Q_nomB / ((param.V_nom-5.6) * sqrt(param.p_ref));
% param.Cv_PtoB  =param.Cv_PtoA  ;
param.Cv_BtoT       = Q_B             / ((4.4 - 2.5)       * sqrt(p_B));
param.Cv_AtoT       = Q_A             / ((param.V_nom-5.6) * sqrt(p_A));

param.Cl_1          = 2/60000;
param.ClH           = 1/100;

param.CvA           = param.Q_nomA / (12.6 * sqrt(param.p_ref));
param.CvB           = param.CvA;

param.Cl1           = 0 * 1e-5 / 60000;
param.Cl2           = 0 * 2    / 60000;
param.Cu            = 0 * 3.2e-10;
param.Cu_0          = 0 * 3.2e-10;