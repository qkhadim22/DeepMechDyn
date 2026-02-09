function [param] = Get_parameters_Patu()
%% Degrees of freedom
param.ndof          = 1;
param.nqdof         = 3;
param.tot_dof       = 2;
param.Index_ind     = [1 4];
param.Index_dep     = [2 3];

%% Constants
param.g             = 9.8066;

%% Valve parameters

% param.valve_models  = ["PVG32_old"   "PVG32_flow_basic" "PVG32_flow_assymetricDB"];

param.valve_models  = ["PVG32_flow_assymetricDB"];


param.V_nom         = 7.5;
param.u_dl          = 5.20;      % 5.4
param.u_dh          = 6.2;       % 6.6

param.Q_nomA        = 62/60000;  % l/min
param.Q_nomB        = 72/60000;  % l/min

Q_A                 = 95/60000;
p_A                 = 53.5e5;

Q_B                 = 145/60000;
p_B                 = 35e5;

param.Dp            = 7e5;
param.p_ref         = 7e5;

param.Cv_PtoA       = param.Q_nomA / ((4.4 - 2.5)       * sqrt(param.p_ref));
param.Cv_PtoB       = param.Q_nomB / ((param.V_nom-5.6) * sqrt(param.p_ref));
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

param.u0            = (0.8)/7;%0.8/7
param.p_ref1         = 20e5;
param.Q0            = 40/6e4;
param.Q1            = 40/6e4;
param.Kv0           = (param.Q0) / sqrt(param.p_ref);
param.U_N           = 5.6;
param.Kv1           = (param.Q1) / sqrt(param.p_ref1);


%% Fluid & bulk moduli
param.p_t           = 1e5;
param.Bh            = 500e6; %700e6
param.Bc            = 1.3e11; %2.1e11
param.Bo            = 900e6; %1650e6

%% Cylinder 1 (lift)
param.C1_A1         = (pi/4)*(100e-3)^2;
param.C1_A2         = param.C1_A1 - (pi/4)*(56e-3)^2;
param.H1            = 535e-3;
param.l_cyl         = 0.820;

%% Cylinder 2 (tilt)
param.C2_A1         = (pi/4)*(100e-3)^2;
param.C2_A2         = param.C2_A1 - (pi/4)*(56e-3)^2;
param.H2            = 780e-3;
param.l_cy2         = 1.050;

%% Line volumes
param.VA            = (pi/4)*(12.7e-3)^2 * 1.5;
param.VB            = (pi/4)*(12.7e-3)^2 * 2.0;

%% Friction parameters
param.Fc            = 210;
param.Fs            = 300;
param.sig2          = 330;
param.vs            = 5e-3;

param.V_aLift       = 0.025e-3;
param.V_aTilt       = 0.01e-3;

%% Motion timing
dt                  = 16/5;
dp_A                = (70.941 - 63.586)/5;
dp_B                = (3.9288 - 2.6478)/5;

param.TMotionExtendingStart   = 2.05 * dt;
param.TMotionExtendingFinish  = 16   + 3.05 * dt;
param.TRampMotion             = 0.05 * dt;
param.TMotionRetractingStart  = 32   + 0.4  * dt;
param.TMotionRetractingFinish = 32   + 2.05 * dt;

%% Pressure profiles
param.pAHold_0                = 63.586e5 + 1.7 * dp_A;
param.pAHold_1                = 63.586e5 - 1.0 * dp_A;

param.pBHold_0                = 0.0865e5 + 3.0 * dp_B;
param.pBHold_1                = 3.9288e5 + 0.25 * dp_B;

param.pAExtendingStart        = 70.941e5 + dp_A;
param.pAExtendingFinish       = 63.586e5 + 3.0 * dp_A;

param.pBExtendingStart        = param.pBHold_0;
param.pBExtendingFinish       = param.pBHold_1;

param.pARetractingStart       = param.pAHold_1;
param.pARetractingFinish      = 63.586e5 + 1.6 * dp_A;

param.pBRetractingStart       = param.pBHold_1;
param.pBRetractingFinish      = param.pBHold_0;











