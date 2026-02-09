function [p1_dot,p2_dot] = Deri_pressure_calculation_lift_Cylinder(param,t, U1,p1,p2,s1,dots1,Va1,Vb1,A1,A2,pP,p_t, i)

% [Qd1,Qd2] = Directional_Flowrates_Valve_Lift(U1,pP,p1,p2);
valve = param.valve_models(i);
 if valve == "PVG32_old" 
        [Qd1,Qd2,~] = PVG32_old(U1,p1,p2,pP,p_t);
 elseif valve == "PVG32_flow_basic" 
        [Qd1,Qd2,~] = PVG32_flow_basic(t,U1);   
 elseif valve == "PVG32_flow_assymetricDB"
        [Qd1,Qd2,~] = PVG32_flow_assymetricDB(t,U1,p1,p2,pP,p_t); % or another function
 end


l1 = s1 - param.H1;
l2 = param.l_cyl - l1;

Vh1        = Va1+A1*l1;
Vh2        = Vb1+A2*l2;

% Calculate the effective bulk modulus
[Be1,Be2]     = EffectiveBulkModulus(s1,l1, l2); %-0.82

% Derivatives of pressures
p1_dot         = (Be1)/(Vh1)*(Qd1-A1*dots1);       % dotp_1
p2_dot         = (Be2)/(Vh2)*(A2*dots1-Qd2);       % dotp_2
