function [p3_dot,p4_dot] = Deri_pressure_calculation_tilt_Cylinder(param,t, U2,p3,p4,s2,dots2,Va1,Vb1,A1,A2,pP, p_t)

if param.DCV
   Cv1 = param.CvB;
   Cv2 = param.CVh2;

   [Qd3,Qd4,~] = DCV43(U2,p3,p4,pP,p_t,Cv1,Cv2 );  
else
   if param.PVG32_old 
    [Qd3,Qd4,~] = PVG32_old(U2,p3,p4,pP,p_t);
   elseif param.PVG32_flow_basic
    [Qd3,Qd4,~] = PVG32_flow_basic(t,U2,p3,p4,pP,p_t);
   else
    [Qd3,Qd4,~] = PVG32_flow_assymetricDB(t,U2,p3,p4,pP,p_t); % or another function
    end
end



l1 = s2 - param.H2;
l2 = param.l_cy2 - l1;

Vh1        = Va1+A1*l1;
Vh2        = Vb1+A2*l2;

% Calculate the effective bulk modulus
[Be1,Be2]     = EffectiveBulkModulus(s2,l1, l2);

% Derivatives of pressures
p3_dot         = (Be1)/(Vh1)*(Qd3-A1*dots2);       % dotp_3
p4_dot         = (Be2)/(Vh2)*(A2*dots2-Qd4);       % dotp_4
