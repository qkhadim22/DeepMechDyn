function [bodies,joints,pos,param_bod] = Get_bodies_PATU_model_semi()

[param]                       = Get_parameters_Patu();


% Body 1
m1              = 124.565967  ;
L1              = 2879*1e-3+23e-2;
g1x             =  1.12015296e+00;
g1y             = 3.02029478e-02;
I1_zz           = 2.50159812e+02;

% Body 2
m2              = 11.524039;
L2              = 452.227e-3;
g2x             = 0.257068;
g2y             = 0.004;
I2_zz           = 0.268644;

% Body 3
m3              = 7.900191;
L3              = 0.48;
g3x             = 0.267208;
g3y             = 0;
I3_zz           = 0.216772;

% Body 4 and payload
m4              = 76.6995;   % Tilt boom mass
mL              = 191.431+20; 
meXT            = 50.7193+7.9; %58.63,50.7193
geXT_x          = 1.21;
geXT_y          = 0.0;

L4              = 2.208;             % X distance from the closed loop, +0.33+0.2
Lcx             = 95e-3;       % Y distance from the closed loop
Lcy             = -243.1993e-3;
g4x             = 4.97072944e-01;               % X-center of mass: tilt boom
g4y             = 2.39420940e-01;               % Y-center of mass: tilt boom

% g4y             = 3.16e-17;               % Y-center of mass: tilt boom
gLx             = L4-Lcx;              % X-center of mass: Payload
gLy             = 0.26;                 % Y-center of mass: Payload 
gcx             = (m4*g4x+meXT*geXT_x+mL*gLx)/(m4+mL+meXT);% X-center of mass: Parallel axis theorem 
gcy             = (m4*g4y+meXT*geXT_y+mL*gLy)/(m4+mL+meXT);
I4_zz           = 5.14527395e+01;  % distance^2*m
IL_zz           = (1/12)*(mL)*((0.25)^2+(0.25)^2);

Ic_zz           = I4_zz+m4*(g4x-gcx)^2+m4*(g4y-gcy)^2+IL_zz+mL*(gLx-gcx)^2 +mL*(gLy-gcy)^2+13;

%%
mass_all        = [m1       m2            m3               m4+mL+meXT];   
L_Length        = [L1       L2            L3                L4];
Inertia_all     = [I1_zz    I2_zz        I3_zz            Ic_zz]; 
nb              = length(L_Length);

for i = 1:nb
    bodies(i).L1 = L_Length(i);
    bodies(i).J = Inertia_all(i); %Inertia_all(i);%1/12*bodies(i).L1^2
    bodies(i).m = mass_all(i);
end


param_bod.u1_bar_j1_o1x = g1x;
param_bod.u1_bar_j1_o1y = g1y;
param_bod.u1_bar_j1_j2x = 2685e-3; 
param_bod.u1_bar_j1_j2y = 0.15e-03;    
param_bod.u2_bar_j2_o2x = g2x;
param_bod.u2_bar_j2_o2y = g2y;
param_bod.u2_bar_j2_j3x = 452.227e-3;         
param_bod.u2_bar_j2_j3y = -42.5e-3;
param_bod.u3_bar_j3_o3x = g3x;          
param_bod.u3_bar_j3_o3y = g3y;
param_bod.u1_bar_j1_j4x = 2875*1e-3;
param_bod.u1_bar_j1_j4y = 15.15e-3;
param_bod.u4_bar_j4_o4x = gcx;
param_bod.u4_bar_j4_o4y = gcy;
%  

joints(1).type = 'revolute';
joints(1).ibody = 3;
joints(1).jbody = 4;
joints(1).bar_ibody = [L3-g3x; -g3y];%[bodies(joints(1).ibody).L1/2; 0];
joints(1).bar_jbody = [-gcx-Lcx;-gcy-Lcy];%[-0.9749; -0.0269];
                        
pos.bar_hy1_up_1 = [304.19*1e-3-g1x; -100.01*1e-3 - g1y];
pos.bar_hy1_up_0 = [0.26 -1426.1*1e-3+386.113249*1e-3+2e-3]';
% pos.bar_hy1_up_0 = [0.26 -1426.1*1e-3+386.113249*1e-3-5e-3]';

pos.R0 = [0.2697;-1.6067];

pos.bar_hy2_up_1 = [1258e-3-g1x; 195e-3 - g1y];%[-0.1673 0.1858]';
pos.bar_hy2_up_2 = [452.227e-3-g2x -42.5e-3-g2y+22e-3]';
pos.bar_weig_4 = [1970.2/1000-0.754827; % add mass
10.565/1000+0.003897];

end


