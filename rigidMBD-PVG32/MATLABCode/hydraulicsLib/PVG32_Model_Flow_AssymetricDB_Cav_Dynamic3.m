close all;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Take data from experiments
%Focus on extraction
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Time reference
dt = 16/5;
TRampUpStart_1 = 1.75*dt; %Start ramp up to extension
TRampUpFinish_1 = 2.1*dt; %Finished rump up to extension
TRampDownStart_1 = 16 + 2.95*dt; %Start ramp down at extension
TRampDownFinish_1 = 16 + 3.3*dt; %Finished ramp down at extension
TRampUpStart_2 = 32; %Start ramp up to retraction
TRampUpFinish_2 = 32 + 0.5*dt; %Finished rump up to retraction
TRampDownStart_2 = 32 + (1 + 11/12.5)*dt; %Start ramp down at retraction
TRampDownFinish_2 = 32 + (2 + 5/12.5)*dt; %Finished ramp down at retraction
%Piston travel
dx = (1141 - 1034)/5;
x_1 = 927 + 0.8*dx;
x_2 = 1141 + 3.1*dx;
x_3 = 927 + 1.3*dx;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pressure A-port
dp_A = (70.941 - 63.586)/5;
pAHold_0 = 63.586 + 1.7*dp_A;
pAExtendingStart = 70.941 + dp_A; %Add friction
%pAInterExtending = 70.941;
pAExtendingFinish = 63.586 + 3*dp_A;
pAHold_1 = 63.586 - 1*dp_A;
pARetractingStart = pAHold_1;
pARetractingFinish = 63.586 + 1.6*dp_A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pressure B-port
dp_B = (3.9288 - 2.6478)/5;
pBHold_0 = 0.0865 + 3*dp_B;
pBExtendingStart = pBHold_0;
pBExtendingFinish = 3.9288 + 0.25*dp_B;
pBHold_1 = pBExtendingFinish;
pBRetractingStart = pBHold_1;
pBRetractingFinish = pBHold_0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Velocities from positions
%Cylinder position
dx = (1248 - 1141)/1e3/5;
xInit_1 = 0.927 + 0.8*dx;
xFinal_1 = 1.141 + 3.1*dx;
xInit_2 = xFinal_1;
xFinal_2 = 0.927 + 1.25*dx;
%Delta times
TMotionExtendingStart = 2.05*dt;
TMotionExtendingFinish = 16 + 3.05*dt;
TRampMotion = 0.05*dt;
TMotionRetractingStart = 32 + 0.4*dt;
TMotionRetractingFinish = 32 + 2.05*dt;
DeltaT_RampUp_1 = TRampUpFinish_1 - TRampUpStart_1;
DeltaT_RampDown_1 = TRampDownFinish_1 - TRampDownStart_1;
DeltaT_RampUp_2 = TRampUpFinish_2 - TRampUpStart_2;
DeltaT_RampDown_2 = TRampDownFinish_2 - TRampDownStart_2;
%Velocities
v0_Average_Measured_1 = (xFinal_1 - xInit_1)/(TMotionExtendingFinish - TMotionExtendingStart);
v0_Max_Measured_1 = (xFinal_1 - xInit_1)/(TMotionExtendingFinish - TMotionExtendingStart - TRampMotion);
v0_Average_Measured_2 = (xFinal_2 - xInit_2)/(TMotionRetractingFinish - TMotionRetractingStart);
v0_Max_Measured_2 = (xFinal_2 - xInit_2)/(TMotionRetractingFinish - TMotionRetractingStart - TRampMotion);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Spool signal
dU = (6.45 - 5.55)/9;
U_Start_1 = 5.55 + 2.5*dU;
U_Hold_1 = 5.55 - 5.5*dU;
U_End_1 = U_Start_1;
U_Start_2 = U_End_1;
U_Hold_2 = 6.45 + 1.5*dU;
U_End_2 = U_Start_2;
%%%%%%%%%%%%%%
%Cylinder data
%%%%%%%%%%%%%%
d = 100/1e3;
dR = 56/1e3;
A = pi*d^2/4;
AR = pi*dR^2/4;
Aa = A - AR;
phi = Aa/A;
%%%%%%%%%%%%%%
%Valve data
%%%%%%%%%%%%%%
Q0 = 40/6e4;
u0 = 0.8/7;
p0 = 7e5;
Kv0 = Q0/p0^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%
%Steady state simulation
%%%%%%%%%%%%%%%%%%%%%%%%
t = 0;
Delta_t = 0.02;
i = 1;
while t < TRampDownFinish_2
    %%%%%%%%%%%%%%%%%%%%%%%
    %Measure voltage signal
    %%%%%%%%%%%%%%%%%%%%%%%
    if t < TRampUpStart_1
        U = U_Start_1;
    elseif t < TRampUpFinish_1
        um = (t - TRampUpStart_1)/DeltaT_RampUp_1;
        U = U_Start_1 + um*(U_Hold_1 - U_Start_1);
    elseif t < TRampDownStart_1
        U = U_Hold_1;
    elseif t < TRampDownFinish_1
        um = (t - TRampDownStart_1)/DeltaT_RampDown_1;
        U = U_End_1 + (1 - um)*(U_Hold_1 - U_End_1);
    elseif t < TRampUpStart_2
        U = U_End_1;
    elseif t < TRampUpFinish_2
        um = (t - TRampUpStart_2)/DeltaT_RampUp_2;
        U = U_Start_2 + um*(U_Hold_2 - U_Start_2);
    elseif t < TRampDownStart_2
        U = U_Hold_2;
    elseif t < TRampDownFinish_2
        um = (t - TRampDownStart_2)/DeltaT_RampDown_2;
        U = U_End_2 + (1 - um)*(U_Hold_2 - U_End_2);
    else
        U = U_End_2;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %Measured position
    %%%%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        LMeasured = xInit_1;
    elseif t < TMotionExtendingFinish
        um = (t - TMotionExtendingStart)/(TMotionExtendingFinish - TMotionExtendingStart);
        LMeasured = xInit_1 + um*(xInit_2 - xInit_1);
    elseif t < TMotionRetractingStart
        LMeasured = xInit_2;
    elseif t < TMotionRetractingFinish
        um = (t - TMotionRetractingStart)/(TMotionRetractingFinish - TMotionRetractingStart);
        LMeasured = xInit_2 + um*(xFinal_2 - xInit_2);
    else
        LMeasured = xFinal_2;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %Measured velocity
    %%%%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        vMeasured = 0;
    elseif t < TMotionExtendingStart + TRampMotion
        um = (t - TMotionExtendingStart)/TRampMotion;
        vMeasured = um*v0_Max_Measured_1;
    elseif t < TMotionExtendingFinish - TRampMotion
        vMeasured = v0_Max_Measured_1;
    elseif t < TMotionExtendingFinish
        um = (t - (TMotionExtendingFinish - TRampMotion))/TRampMotion;
        vMeasured = (1 - um)*v0_Max_Measured_1;
    elseif t < TMotionRetractingStart
        vMeasured = 0;
    elseif t < TMotionRetractingStart + TRampMotion
        um = (t - TMotionRetractingStart)/TRampMotion;
        v_Measured = um*v0_Max_Measured_2;
    elseif t < TMotionRetractingFinish - TRampMotion
        vMeasured = v0_Max_Measured_2;
    elseif t < TMotionRetractingFinish
        um = (t - (TMotionRetractingFinish - TRampMotion))/TRampMotion;
        vMeasured = (1 - um)*v0_Max_Measured_2;
    else
        vMeasured = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%
    %Measured port pressures
    %%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        pAMeasured = pAHold_0;
        pBMeasured = pBHold_0;
    elseif t < TMotionExtendingFinish
        um = (t - TMotionExtendingStart)/(TMotionExtendingFinish - TMotionExtendingStart);
        pAMeasured = pAExtendingStart + um*(pAExtendingFinish - pAExtendingStart);
        pBMeasured = pBExtendingStart + um*(pBExtendingFinish - pBExtendingStart);
    elseif t < TMotionRetractingStart
        pAMeasured = pAHold_1;
        pBMeasured = pBHold_1;
    elseif t < TMotionRetractingFinish
        um = (t - TMotionRetractingStart)/(TMotionRetractingFinish - TMotionRetractingStart);
        pAMeasured = pARetractingStart + um*(pARetractingFinish - pARetractingStart);
        pBMeasured = pBRetractingStart + um*(pBRetractingFinish - pBRetractingStart);
    else
        pAMeasured = pAHold_0;
        pBMeasured = pBHold_0;
    end
    %SI-units
    pAMeasured = pAMeasured*1e5;
    pBMeasured = pBMeasured*1e5;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Force based on measured port pressures
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Friction force
    Ffr = ((pAExtendingStart*A - pBExtendingStart*Aa)*1e5 - (pAHold_0*A - pBHold_0*Aa)*1e5)/2;
    FLP = pAMeasured*A - pBMeasured*Aa;
    if t < TMotionExtendingStart
        FL = FLP + Ffr;
    elseif t < TMotionExtendingFinish
        FL = FLP - Ffr;
    elseif t < TMotionRetractingStart
        FL = FLP + Ffr;
    elseif t < TMotionRetractingFinish
        FL = FLP + Ffr;
    else
        FL = FLP + Ffr;
    end
    %%%%%%%%%%%%%%%%%%%%%%%
    %Compute flow assuming
    %neutral @5.75V (assymetrical)
    %0.8/7 deadband to both sides
    %Assuming spool at correct position
    u = (5.8 - U)/3;
    if u > u0 %Extending
        Qin = (u - u0)/(1 - u0)*Q0;
        Kv = (u - u0)/(1 - u0)*Kv0;
    elseif u > -u0 %Neutral
        Qin = 0;
        Kv = 1e-10;
    else %Retracting
        Qin = -(abs(u) - u0)/(1 - u0)*Q0;
        Kv = (abs(u) - u0)/(1 - u0)*Kv0;
        Qin_ALT = -Kv*(pAMeasured*1e5)^0.5;
    end
    %Compute velocity
    if Qin >= 0
        v = Qin/A;
        v_ALT = v;
    else
        v = Qin/Aa;
        v_ALT = Qin_ALT/A;
    end
    %Save data for plotting
    t_Plot(i) = t;
    U_Plot(i) = U;
    v_Plot(i) = v;
    v_ALT_Plot(i) = v_ALT;
    pAMeasured_Plot(i) = pAMeasured/1e5; %bar
    pBMeasured_Plot(i) = pBMeasured/1e5; %bar
    vMeasured_Plot(i) = vMeasured;
    LMeasured_Plot(i) = LMeasured;
    FLP_Plot(i) = FLP/1e3; %kN
    FL_Plot(i) = FL/1e3; %kN
    %Increase time and counter
    t = t + Delta_t;
    i = i + 1;
end
% %Plotting
% figure;
% plot(t_Plot, v_Plot);
% hold on;
% plot(t_Plot, vMeasured_Plot, 'r');
% plot(t_Plot, v_ALT_Plot, 'g');
% grid;
% figure;
% plot(t_Plot, U_Plot, 'r');
% grid;
% figure;
% plot(t_Plot, pAMeasured_Plot);
% hold on;
% plot(t_Plot, pBMeasured_Plot, 'r');
% grid;
% figure;
% plot(t_Plot, FLP_Plot);
% hold on;
% plot(t_Plot, FL_Plot, 'r');
% grid;
% figure;
% plot(t_Plot, LMeasured_Plot);
% grid;
% 
% %Save data
% xData = [t_Plot' U_Plot' v_Plot' v_ALT_Plot' vMeasured_Plot' pAMeasured_Plot' pBMeasured_Plot'];
% 
% 
% dlmwrite('xData.txt', xData, 'delimiter', '\t');  % Tab-delimited text file
% 


%%%%%%%%%%%%%%%%%%%%%%%%
%Dynamic simulation
%%%%%%%%%%%%%%%%%%%%%%%%
U_0 = 5.77; %Neutral voltage
h = 0.6; %Assumed cylinder stroke
LDB = 0.2; %Assumed deadlength of cylinder
xMin = h + LDB;
xMax = 2*h + LDB;

%External load as a function of position, FL_0 to FL_1

%Friction load, Ffr
FfrMax = ((pAExtendingStart*A - pBExtendingStart*Aa)*1e5 - (pAHold_0*A - pBHold_0*Aa)*1e5)/2;
%vTanh = 0.0001; %Shape parameter for friction force around zero velocity

%Effective mass and external force vs. cylinder position
FL_0 = (pAExtendingStart*A - pBExtendingStart*Aa)*1e5 - FfrMax; %External force retracted position
FL_0_Alt = (pAHold_0*A - pBHold_0*Aa)*1e5 + FfrMax;
FL_1 = (pAExtendingFinish*A - pBExtendingFinish*Aa)*1e5 - FfrMax; %External force extracted position
%Approximate cylinder gearing, iG
%Simulating as if we have a slightly varying m_PL*g*iG gravitational load on the cylinder
%Simulating as if we have a certain gearing iG so that the effective
%inertia mass mEff is m_PL*iG^2
iG = 4; %Assumed gearing to compute effective mass (could be larger)
g = 9.81;
m_PL = (FL_0 + FL_1)/2/iG/g; %Average virtual gravitationally loaded mass during stroke
mEff = m_PL*iG^2; %Average efficient mass
m_PL0 = FL_0/iG/g; %Virtual gravitationally loaded mass at beginning of stroke
m_PL1 = FL_1/iG/g; %Virtual gravitationally loaded mass at end of stroke


%Liquid parameters
rho = 875;
beta = 1000e6; % beta = 1000 MPa - good approximation 96% of this beta @60bar
VLA = 5e-4; %Line volume, half a liter (quite high)
VLB = 0.015; %Virtual volume (accumulator like behavior) on B-side
pMin = pBHold_0*1e5; %Assuming return pressure (T-port) at this level near PVG32

%Initialization
%Four state variables
pA = pAMeasured_Plot(1)*1e5; %SI-units
pB = pBMeasured_Plot(1)*1e5; %SI-units
x = xInit_1 - h - LDB; %Piston travel
xDot = 0; %Start from rest

%Empty parameters
tPlot = [];
pAPlot = [];
pBPlot = [];
xPlot = [];
xDotPlot = [];
pAMeasured_Plot = [];
pBMeasured_Plot = [];
vMeasured_Plot = [];
LMeasured_Plot = [];

%Time and counter parameters
t = 0;
Delta_t = 1e-5;
i = 1;
while t < TRampDownFinish_2
    %%%%%%%%%%%%%%%%%%%%%%%
    %Measured voltage signal
    %%%%%%%%%%%%%%%%%%%%%%%
    if t < TRampUpStart_1
        U = U_Start_1;
    elseif t < TRampUpFinish_1
        um = (t - TRampUpStart_1)/DeltaT_RampUp_1;
        U = U_Start_1 + um*(U_Hold_1 - U_Start_1);
    elseif t < TRampDownStart_1
        U = U_Hold_1;
    elseif t < TRampDownFinish_1
        um = (t - TRampDownStart_1)/DeltaT_RampDown_1;
        U = U_End_1 + (1 - um)*(U_Hold_1 - U_End_1);
    elseif t < TRampUpStart_2
        U = U_End_1;
    elseif t < TRampUpFinish_2
        um = (t - TRampUpStart_2)/DeltaT_RampUp_2;
        U = U_Start_2 + um*(U_Hold_2 - U_Start_2);
    elseif t < TRampDownStart_2
        U = U_Hold_2;
    elseif t < TRampDownFinish_2
        um = (t - TRampDownStart_2)/DeltaT_RampDown_2;
        U = U_End_2 + (1 - um)*(U_Hold_2 - U_End_2);
    else
        U = U_End_2;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %Measured position
    %%%%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        LMeasured = xInit_1;
    elseif t < TMotionExtendingFinish
        um = (t - TMotionExtendingStart)/(TMotionExtendingFinish - TMotionExtendingStart);
        LMeasured = xInit_1 + um*(xInit_2 - xInit_1);
    elseif t < TMotionRetractingStart
        LMeasured = xInit_2;
    elseif t < TMotionRetractingFinish
        um = (t - TMotionRetractingStart)/(TMotionRetractingFinish - TMotionRetractingStart);
        LMeasured = xInit_2 + um*(xFinal_2 - xInit_2);
    else
        LMeasured = xFinal_2;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %Measured velocity
    %%%%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        vMeasured = 0;
    elseif t < TMotionExtendingStart + TRampMotion
        um = (t - TMotionExtendingStart)/TRampMotion;
        vMeasured = um*v0_Max_Measured_1;
    elseif t < TMotionExtendingFinish - TRampMotion
        vMeasured = v0_Max_Measured_1;
    elseif t < TMotionExtendingFinish
        um = (t - (TMotionExtendingFinish - TRampMotion))/TRampMotion;
        vMeasured = (1 - um)*v0_Max_Measured_1;
    elseif t < TMotionRetractingStart
        vMeasured = 0;
    elseif t < TMotionRetractingStart + TRampMotion
        um = (t - TMotionRetractingStart)/TRampMotion;
        v_Measured = um*v0_Max_Measured_2;
    elseif t < TMotionRetractingFinish - TRampMotion
        vMeasured = v0_Max_Measured_2;
    elseif t < TMotionRetractingFinish
        um = (t - (TMotionRetractingFinish - TRampMotion))/TRampMotion;
        vMeasured = (1 - um)*v0_Max_Measured_2;
    else
        vMeasured = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%
    %Measured port pressures
    %%%%%%%%%%%%%%%%%%%%%%%
    if t < TMotionExtendingStart
        pAMeasured = pAHold_0;
        pBMeasured = pBHold_0;
    elseif t < TMotionExtendingFinish
        um = (t - TMotionExtendingStart)/(TMotionExtendingFinish - TMotionExtendingStart);
        pAMeasured = pAExtendingStart + um*(pAExtendingFinish - pAExtendingStart);
        pBMeasured = pBExtendingStart + um*(pBExtendingFinish - pBExtendingStart);
    elseif t < TMotionRetractingStart
        pAMeasured = pAHold_1;
        pBMeasured = pBHold_1;
    elseif t < TMotionRetractingFinish
        um = (t - TMotionRetractingStart)/(TMotionRetractingFinish - TMotionRetractingStart);
        pAMeasured = pARetractingStart + um*(pARetractingFinish - pARetractingStart);
        pBMeasured = pBRetractingStart + um*(pBRetractingFinish - pBRetractingStart);
    else
        pAMeasured = pAHold_0;
        pBMeasured = pBHold_0;
    end
    %SI-units
    pAMeasured = pAMeasured*1e5;
    pBMeasured = pBMeasured*1e5;
    %%%%%%%%%%%%%%%%%%%%%%%
    %Compute flow assuming
    %neutral @5.77V (assymetrical)
    %0.8/7 deadband to both sides
    %Assuming spool at correct position
    u = (U_0 - U)/3;
    if u > u0 %Extending
        QA = (u - u0)/(1 - u0)*Q0;
        Kv = (u - u0)/(1 - u0)*Kv0;
        QB = -Kv*(pB - pMin)^0.5;
    elseif u > -u0 %Neutral
        Kv = 1e-10; %Very small leakage
        QA = - Kv*(pA - pMin)^0.5 - sign(pA - pB)*Kv*abs(pA - pB)^0.5;
        QB = sign(pA - pB)*Kv*abs(pA - pB)^0.5-Kv*(pB - pMin)^0.5;
    else %Retracting
        QB = (abs(u) - u0)/(1 - u0)*Q0;
        Kv = (abs(u) - u0)/(1 - u0)*Kv0;
        QA = -Kv*(pA - pMin)^0.5;
    end
    %%%%%%%%%%%%%%%%%%%%
    % Pressure gradient
    %%%%%%%%%%%%%%%%%%%%
    pBeta = 30e5; %Full beta (96%) at 60 bar
    betaMin = 2;
    betaMax = 1000e6;
    uBeta = pA/pBeta;
    if uBeta > 1
        beta = betaMax;
    else
        beta = (3*uBeta^2 - 2*uBeta^3)*betaMax;
    end
    pADot = (QA - xDot*A)*beta/(VLA + x*A);
    uBeta = pB/pBeta;
    if uBeta > 1
        beta = betaMax;
    else
        beta = (3*uBeta^2 - 2*uBeta^3)*betaMax;
    end
    pBDot = (QB + xDot*Aa)*beta/(VLB + (h - x)*A);
    %%%%%%%%%%%%%%%%%%%%
    % Acceleration
    %%%%%%%%%%%%%%%%%%%%
    %Linear interpolation of m_PL
    m_PL = m_PL0 + (x - (xInit_1 - h - LDB))/(xFinal_1 - xInit_1)*(m_PL1 - m_PL0);
    FL = m_PL*g*iG;
%    Ffr = FfrMax*tanh(xDot/vTanh);
    vEps = 1e-5;
    if xDot > vEps
        Ffr = FfrMax;
    else
        Ffr = -FfrMax;
    end
    bCyl = 1e4;
    Fvisc = bCyl*xDot;
    xDotDot = (pA*A - pB*Aa - FL - Ffr - Fvisc)/mEff;
    %%%%%%%%%%%%%%%%%%%%%%
    % Save data for plotting
    %%%%%%%%%%%%%%%%%%%%%%
    tPlot(i) = t;
    xPlot(i) = x;
    xDotPlot(i) = xDot;
    pAPlot(i) = pA/1e5;
    pBPlot(i) = pB/1e5;
    pAMeasured_Plot(i) = pAMeasured/1e5; %bar
    pBMeasured_Plot(i) = pBMeasured/1e5; %bar
    vMeasured_Plot(i) = vMeasured;
    xMeasured_Plot(i) = LMeasured - LDB - h;
    Ffr_Plot(i) = Ffr;
    %%%%%%%%%%%%%%%%%%%%%%
    % Time integration
    %%%%%%%%%%%%%%%%%%%%%%
    pA = pA + pADot*Delta_t;
    if pA < pMin;
        pA = pMin;
    end
    pB = pB + pBDot*Delta_t;
    if pB < pMin;
        pB = pMin;
    end
    x = x + xDot*Delta_t;
    xDot = xDot + xDotDot*Delta_t;
    t = t + Delta_t;
    i = i + 1;
end

% figure;
% plot(tPlot, xDotPlot);
% grid;
figure;
plot(tPlot, pBPlot);
hold on;
plot(tPlot, pBMeasured_Plot, 'r');
grid;
figure;
plot(tPlot, pAPlot);
hold on;
plot(tPlot, pAMeasured_Plot, 'r');
grid;
figure;
plot(tPlot, xPlot);
hold on;
plot(tPlot, xMeasured_Plot, 'r');
grid;