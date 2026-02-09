function [Q_hyd, p_hyd_dot, s1, dots1, s2, dots2] = Hydralic_force_Patu_rigid(t, R,phi,dR,dphi,p_hyd,param,U1,U2,pos,pP,p_t, I)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Lift Hydralic force *******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1 = p_hyd(1);
p2 = p_hyd(2);
R1 = R(:,1);    %g1
phi1 = phi(1);  %z1
dR1 = dR(:,1);  %global dg1
dphi1 = dphi(1);% gloabl dz1 
A1     = param.C1_A1;
A2     = param.C1_A2;
Fc     = param.Fc;
Fs     = param.Fs;
vs     = param.vs;
sig2   = param.sig2;
Cl1    = param.Cl1;
Cl2    = param.Cl2;


%%%% Hydralic force
[Qe_1, s1, dots1]  = Force_calculation_lift_Cylinder(t, R1,phi1,dR1,dphi1,p1,p2,A1,A2,Fs,Fc,vs,sig2,pos, Cl1);

Va1    = param.VA;
Vb1    = param.VA;
% Derivatives of pressures
[p1_dot,p2_dot] = Deri_pressure_calculation_lift_Cylinder(param, t, U1,p1,p2,s1,dots1,Va1,Vb1,A1,A2,pP,p_t, I);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Tilt Hydralic force *******
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3 = p_hyd(3);
p4 = p_hyd(4);
R2 = R(:,2);
phi2 = phi(2);
dR2 = dR(:,2);
dphi2 = dphi(2);
A1     = param.C2_A1;
A2     = param.C2_A2;
Fc     = param.Fc;
Fs     = param.Fs;
vs     = param.vs;
sig2   = param.sig2;

%%%% Hydralic force
[Qe_12,Qe_2, s2, dots2] = Force_calculation_tilt_Cylinder(R1,phi1,R2,phi2,dR1,dphi1,dR2,dphi2,p3,p4,A1,A2,Fs,Fc,vs,sig2,pos, Cl2);

Va1    = param.VB;
Vb1    = param.VB;
% Derivatives of pressures
[p3_dot,p4_dot] = Deri_pressure_calculation_tilt_Cylinder(param,t, U2,p3,p4,s2,dots2,Va1,Vb1,A1,A2,pP,p_t,I);


Q_hyd = [Qe_1+Qe_12;
     Qe_2;
    zeros(6,1)];

p_hyd_dot = [p1_dot;
            p2_dot;
            p3_dot;
            p4_dot];










