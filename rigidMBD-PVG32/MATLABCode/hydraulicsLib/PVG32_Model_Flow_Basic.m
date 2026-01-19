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
%Pressure A-port
dp_A = (70.941 - 63.586)/5;
pA_Start_1 = 63.586 + 1.7*dp_A;
pA_Start_Extending = 70.941 + dp_A; %Add friction
pA_Inter_Extending = 70.941;
pA_End_Extending = 63.586 + 3*dp_A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%pA extending approximated as polynomium
t_Inter = 16 - TRampUpStart_1;
t_End = TRampDownFinish_1 - TRampUpStart_1;
u_Inter = t_Inter/t_End;
SysMat = [1 0 0; 1 u_Inter u_Inter^2; 1 1 1];
SysRight = [pA_Start_Extending pA_Inter_Extending pA_End_Extending]';
SysC = inv(SysMat)*SysRight;
A_pA = SysC(1);
B_pA = SysC(2);
C_pA = SysC(3);
%Friction pressure
dp_Fric = 4*dp_A;
%Pressure A-port holding
pA_Hold = pA_End_Extending - dp_Fric;
pA_Start2 = pA_Hold;
pA_Start_Retracting = pA_Start2;
pA_End_Retracting = pA_Start_Extending - dp_Fric;
%Pressure B-port
dp_B = (3.9288 - 2.6478)/5;
pB_Start = 1.3667 - 2.5*dp_B;
pB_End = 3.9288 + 0.3*dp_B;
%Spool signal
dU = (6.45 - 5.55)/9;
U_Start_1 = 5.55 + 2.5*dU;
U_Hold_1 = 5.55 - 5.5*dU;
U_End_1 = U_Start_1;
U_Start_2 = U_End_1;
U_Hold_2 = 6.45 + 1.5*dU;
U_End_2 = U_Start_2;
%Cylinder speed
dv = (0.0177 - 0.0069)/5;
v0_1 = 0.0069 + 3.2*dv;
% %Cylinder position
% dx = (1248 - 1141)/1e3/5;
% xInit_1 = 0.927 + 0.8*dx;
% xFinal_1 = 1.141 + 3.1*dx;
% xInit_2 = xFinal_1;
% xFinal_2 = 0.927 + 1.25*dx;
% %Velocities from positions
% DeltaT_RampUp_1 = TRampUpFinish_1 - TRampUpStart_1;
% DeltaT_RampDown_1 = TRampDownFinish_1 - TRampDownStart_1;
% DeltaT_RampUp_2 = TRampUpFinish_2 - TRampUpStart_2;
% DeltaT_RampDown_2 = TRampDownFinish_2 - TRampDownStart_2;
% v0_Average_Measured_1 = (xFinal_1 - xInit_1)/(TRampDownFinish_1 - TRampUpStart_1);
% v0_Max_Measured_1 = (xFinal_1 - xInit_1)/(TRampDownFinish_1 - TRampUpStart_1 - DeltaT_RampUp_1/2 - DeltaT_RampDown_1/2);
% v0_Average_Measured_2 = (xFinal_2 - xInit_2)/(TRampDownFinish_2 - TRampUpStart_2);
% v0_Max_Measured_2 = (xFinal_2 - xInit_2)/(TRampDownFinish_2 - TRampUpStart_2 - DeltaT_RampUp_2/2 - DeltaT_RampDown_2/2);
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
TMotionRetractingStart = 32 + 1/3*dt;
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
%Load data
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
%u0 = 1.5/7;
%V0 = u0*3;
p0 = 7e5;
Kv0 = Q0/p0^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%
%Steady state simulation
%%%%%%%%%%%%%%%%%%%%%%%%
t = 0;
Delta_t = 0.02;
i = 1;
while t < TRampDownFinish_2
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %Measured control voltage
    %%%%%%%%%%%%%%%%%%%%%%%%%
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
    %Compute flow assuming
    %neutral @6V
    %0.8/7 deadband symmetrical
    %Assuming spool at correct position
    u = (6 - U)/3;
    if u > u0
        Qin = (u - u0)/(1 - u0)*Q0;
        Kv = (u - u0)/(1 - u0)*Kv0;
    elseif u > -u0
        Qin = 0;
        Kv = 1e-10;
    else
        Qin = -(abs(u) - u0)/(1 - u0)*Q0;
        Kv = (abs(u) - u0)/(1 - u0)*Kv0;
    end
    %Compute velocity
    if Qin >= 0
        v = Qin/A;
    else
        v = Qin/Aa;
    end
    %Save data for plotting
    t_Plot(i) = t;
    U_Plot(i) = U;
    v_Plot(i) = v;
    vMeasured_Plot(i) = vMeasured;
    %Increase time and counter
    t = t + Delta_t;
    i = i + 1;
end
%Plotting
figure;
plot(t_Plot, v_Plot);
hold on;
plot(t_Plot, vMeasured_Plot,'r');
grid;
figure;
plot(t_Plot, U_Plot, 'r');
grid;
 
%Save data
xData = [t_Plot' U_Plot' v_Plot' vMeasured_Plot'];


%dlmwrite('xDataBasic.txt', xData, 'delimiter', '\t');  % Tab-delimited text file