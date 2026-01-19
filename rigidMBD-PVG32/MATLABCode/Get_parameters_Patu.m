function [param] = Get_parameters_Patu()
param.DCV                = false; % make it false for PVG32
param.PVG32_old          = false;  % set true only, if param.DCV=false
param.PVG32_flow_basic   = true; % set true only, if param.PVG32_old=false
param.measurements       = true;

param.ndof = 1;
param.nqdof = 3;
param.tot_dof = 2;
param.Index_ind = [1 4];
param.Index_dep = [2 3];

param.g = 9.8066;


if param.measurements
    if param.DCV
        param.CvA      =  (15/60000)/((9.9)*sqrt(15e5));%param.Q_nomA/((12.6)*sqrt(param.p_ref));
        param.CvB      =(15/60000)/((9.9)*sqrt(7.5e5));
        param.CVh1      =  (24/60000)/((9.9)*sqrt(30e5));%param.Q_nomA/((12.6)*sqrt(param.p_ref));
        param.CVh2      =  (15/60000)/((9.9)*sqrt(7.e5));%param.Q_nomA/((12.6)*sqrt(param.p_ref));


        param.Cl1      = 0*1e-5/60000;
        param.Cl2      = 0*2/60000;
        param.cu_1     = -3.2e-12*0;
        param.cu_2     = -3.2e-10*0;

    else 
        param.V_nom    = 7.5;
        param.u_dl     = 5.20; %5.4
        param.u_dh     = 6.2; %6.6

        param.Q_nomA    = (62/60000); %(40/60000) % may not ok, l/min
        param.Q_nomB    = (72/60000); %(40/60000) % ok, l/min

        Q_B           = (145/60000); % may not ok
        p_B           = 35e5;         % may not ok, pressure difference

        Q_A           = (95/60000);  % Ok
        p_A           = 53.5e5;      % Ok
        param.Dp       = 7e5;
        param.p_ref    = 7e5;

        param.Cv_PtoA  = param.Q_nomA/((4.4-2.5 )*sqrt(param.p_ref));
        param.Cv_PtoB  = param.Q_nomB/((param.V_nom-5.6 )*sqrt(param.p_ref));
         param.Cv_BtoT = Q_B/((4.4-2.5 )*sqrt(p_B));

        param.Cv_AtoT = Q_A/((param.V_nom-5.6 )*sqrt(p_A));
        param.Cl_1      = 2/60000;
        param.ClH      =1/100;

        param.CvA      = param.Q_nomA/((12.6)*sqrt(param.p_ref));
        param.CvB      = param.CvA;
        param.Cl1      = 0*1e-5/60000;
        param.Cl2      = 0*2/60000;
        param.Cu       = 0*3.2e-10;
        param.Cu_0     = 0*3.2e-10;
        param.u0       = 0.8/7; %0.8/7
        param.Q0       = 40/6e4; %40/6e4
        param.Kv0      = param.Q0/sqrt(param.p_ref);

        param.U_0      = 5.77;
    end
else
       param.CvA      = 2.138e-8;
       param.Cl1      = 0*1e-5/60000;
       param.Cl2      = 0*2/60000;
       param.Cu       = 0*3.2e-10;
       param.Cu_0     = 0*3.2e-10;
end


param.p_t      = 1e5;
param.Bh      = 700e6;
param.Bc      = 2.1000e+11;
param.Bo      = 1650e6;

param.C1_A1    =  (pi/4)*(100e-3)^2;                                                   % Cylinder side area ( Cylinder#01)
param.C1_A2    =  param.C1_A1-(pi/4)*(56e-3)^2;                                   % Piston side area  ( Cylinder#01)
param.H1       =  535e-3; % max piston position

%Cylinder 1 for tilt
param.C2_A1    =  (pi/4)*(100e-3)^2;                                                   % Cylinder side area ( Cylinder#01)
param.C2_A2    =  param.C2_A1-(pi/4)*(56e-3)^2;                                   % Piston side area  ( Cylinder#01)

% Volumes between DCV and ports of Cylinder 1
param.VA       =  pi/4*(12.7e-3)^2*1.5;                                                      % Volume of line between DCV and rod side of cylinder 2 (line 3)
param.VB       =  pi/4*(12.7e-3)^2*2;                                                      % Volume of line between DCV and rod side of cylinder 2 (line 3)

%%%%%%%%%%
param.Fc       =  210;  % 210, 1500
param.Fs       =  300;  % 830, 2000
param.sig2     =  330;  % 330, 5000
param.vs       =  5e-3;
param.V_aLift  = 0.025e-3;
param.V_aTilt  = 0.01e-3;


param.H1 = 535e-3;
param.l_cyl = 0.820;

param.H2 = 780e-3;
param.l_cy2 = 1.050;


dt = 16/5;
dp_A = (70.941 - 63.586)/5;
dp_B = (3.9288 - 2.6478)/5;

param.TMotionExtendingStart = 2.05*dt;
param.TMotionExtendingFinish = 16 + 3.05*dt;
param.TRampMotion = 0.05*dt;
param.TMotionRetractingStart = 32 + 0.4*dt;
param.TMotionRetractingFinish = 32 + 2.05*dt;

param.pAHold_0 = 63.586e5 + 1.7*dp_A;
param.pAHold_1 = 63.586e5 - 1*dp_A;
param.pBHold_0 = 0.0865e5 + 3*dp_B;
param.pAExtendingStart = 70.941e5 + dp_A;
param.pAExtendingFinish = 63.586e5 + 3*dp_A;
param.pARetractingStart = param.pAHold_1;
param.pBExtendingStart = param.pBHold_0;
param.pBExtendingFinish = 3.9288e5 + 0.25*dp_B;
param.pBHold_1 = param.pBExtendingFinish;
param.pARetractingFinish = 63.586e5 + 1.6*dp_A;

param.pBRetractingStart = param.pBHold_1;
param.pBRetractingFinish = param.pBHold_0;










