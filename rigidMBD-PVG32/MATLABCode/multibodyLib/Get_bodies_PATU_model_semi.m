function [bodies,joints,pos,param_bod] = Get_bodies_PATU_model_semi(loadCase)
% New numbering:
%   body 1 = lift boom
%   body 2 = tilt boom + extension + payload   (joint J2 = tilt pivot on lift boom)
%   body 3 = bracket 2                         (joint J3 = lug C on tilt boom)
%   body 4 = bracket 1                         (joint J4 = bracket-1 pivot on lift boom)
%   closed loop: pin between bracket 2 and bracket 1
[param]                       = Get_parameters_Patu();

%% Body 1: lift boom
m1              = 124.565967;
L1              = 2879*1e-3+23e-2;
g1x             = 1.12015296e+00;
g1y             = 3.02029478e-02;
I1_zz           = 2.50159812e+02 - m1*(g1x^2+g1y^2);     % CAD about joint -> shifted to CoM


%% Body 4: bracket 1
m2              = 11.524039;                             % bracket 1 mass (variable name kept)
L2              = 457.23e-3;
g2x             = 0.257068;
g2y             = -0.004;
I2_zz           = 0.268644;

%% Body 2: tilt boom + extension + payload
m4              = 76.6995;                               % tilt boom mass (variable name kept)
switch upper(loadCase)
    case "95KG"
        mL      = 95+11.431+13.495;
    case "148KG"
        mL      = 148+11.431+13.495;
    case "180KG"
        mL      = 180+11.431+14.364;
    case "278KG"
        mL      = 278-56.13;
    otherwise
        error("Unknown loadCase: " + loadCase);
end
meXT            = 50.7193+7.9;                           % 58.63, 50.7193
geXT_x          = 1.21;
geXT_y          = 0.0;
L4              = 2.208;                                 % X distance from the closed loop, +0.33+0.2
Lcx             = 95e-3;                                 % tilt pivot -> lug C = (-Lcx, -Lcy) (Excel: -95, 243.199)
Lcy             = -243.1993e-3;

g4x             = 4.97072944e-01;                        % X-center of mass: tilt boom
g4y             = 2.39420940e-01;                        % Y-center of mass: tilt boom
gLx             = L4-Lcx;                                % X-center of mass: payload
gLy             = 0.26;                                  % Y-center of mass: payload
gcx             = (m4*g4x+meXT*geXT_x+mL*gLx)/(m4+mL+meXT);   % combined CoM of body 2
gcy             = (m4*g4y+meXT*geXT_y+mL*gLy)/(m4+mL+meXT);
I4_zz           = 5.14527395e+01 - m4*(g4x^2+g4y^2);     % CAD about joint -> shifted to CoM (= 28.10)
IL_zz           = (1/12)*(mL)*((0.25)^2+(0.25)^2);
IeXT_zz         = 13;                                    % extension: own inertia about its CoM (check this value)
Ic_zz           = I4_zz   + m4  *((g4x-gcx)^2    + (g4y-gcy)^2) ...
                + IeXT_zz + meXT*((geXT_x-gcx)^2 + (geXT_y-gcy)^2) ...
                + IL_zz   + mL  *((gLx-gcx)^2    + (gLy-gcy)^2);

%% Body 3: bracket 2 (local x from bracket1-bracket2 pin towards lug C, as before)
m3              = 7.900191;
L3              = 0.48;                                  % Excel: 480
g3x             = 0.212792;                              % CoM measured from the bracket1-bracket2 pin
g3y             = 0;
I3_zz           = 0.216772;



%% Assemble bodies (new order)
mass_all        = [m1       m4+mL+meXT    m3       m2   ];
L_Length        = [L1       L4            L3       L2   ];
Inertia_all     = [I1_zz    Ic_zz         I3_zz    I2_zz];
nb              = length(L_Length);
for i = 1:nb
    bodies(i).L1 = L_Length(i);
    bodies(i).J  = Inertia_all(i);
    bodies(i).m  = mass_all(i);
end

%% Joint -> CoM vectors (each in its own body frame)
param_bod.u1_bar_j1_o1x = g1x;          param_bod.u1_bar_j1_o1y = g1y;     % lift boom
param_bod.u2_bar_j2_o2x = gcx;          param_bod.u2_bar_j2_o2y = gcy;     % tilt boom (combined)
param_bod.u3_bar_j3_o3x = g3x - L3;     param_bod.u3_bar_j3_o3y = g3y;     % bracket 2, seen from lug C
param_bod.u4_bar_j4_o4x = g2x;          param_bod.u4_bar_j4_o4y = g2y;     % bracket 1

%% Joint -> child joint vectors
param_bod.u1_bar_j1_j2x = 2875e-3;      param_bod.u1_bar_j1_j2y = 15.15e-3;   % lift boom: J1 -> tilt pivot   (Excel 2875, 15.15)
param_bod.u1_bar_j1_j4x = 2685e-3;      param_bod.u1_bar_j1_j4y = 0.15e-3;    % lift boom: J1 -> bracket-1 pivot (Excel 2685, 0.15)
param_bod.u2_bar_j2_j3x = -Lcx;         param_bod.u2_bar_j2_j3y = -Lcy;       % tilt boom: tilt pivot -> lug C

%% Closed-loop joint: bracket 2 (body 3) - bracket 1 (body 4)
joints(1).type      = 'revolute';
joints(1).ibody     = 3;
joints(1).jbody     = 4;
joints(1).bar_ibody = [-g3x; -g3y];                     % pin seen from bracket-2 CoM
joints(1).bar_jbody = [452.227e-3 - g2x; -42.5e-3 - g2y];           % pin seen from bracket-1 CoM
joints(1).T_i       = kron(param.Tz(joints(1).ibody,:), eye(3));
joints(1).T_j       = kron(param.Tz(joints(1).jbody,:), eye(3));

%% Hydraulic attachment points
pos.bar_hy1_up_1 = [304.19*1e-3-g1x; -100.01*1e-3 - g1y];          % lift cyl on body 1
pos.bar_hy1_up_0 = [0.26 -1426.1*1e-3+386.113249*1e-3+2e-3]';      % lift cyl on ground
pos.R0           = [0.2697;-1.6067];
pos.bar_hy2_up_1 = [1258e-3-g1x; 194.59e-3 - g1y];                 % tilt cyl on body 1 (Excel 1258.0, 194.59)
pos.bar_hy2_up_4    = [452.227e-3 - g2x; -42.5e-3 - g2y]; 
pos.bar_weig_4   = [1970.2/1000-0.754827; 10.565/1000+0.003897];
end