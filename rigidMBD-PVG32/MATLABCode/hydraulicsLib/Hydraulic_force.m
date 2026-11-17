function [Fh, F_mu] = Hydraulic_force(param,pA,pB, s, dots,i)

l_A         = s - param.cyl(i).l_cyl;
l_B         = param.cyl(i).l_rod - l_A;

% End-damper force
if l_A <= param.cyl(i).ld
    Fd = param.cyl(i).k*(param.cyl(i).ld - l_A) - param.cyl(i).c*dots;
elseif l_B <= param.cyl(i).ld
    Fd = -param.cyl(i).k*(param.cyl(i).ld - l_B) - param.cyl(i).c*dots;
else
    Fd = 0;
end


% Friction force
% Friction force
param.cyl(i).Fc = param.cyl(i).Fc*(1 + param.cyl(i).Kp*(pA*param.cyl(i).A_cyl-pB*param.cyl(i).A_rod));  
param.cyl(i).Fs = param.cyl(i).Fs*(1 + param.cyl(i).Kp*(pA*param.cyl(i).A_cyl-pB*param.cyl(i).A_rod));
F_mu            = FrictionForce_BrownMcPhee(param,dots,i );

% Net hydraulic force
Fh          = pA*param.cyl(i).A_cyl-pB*param.cyl(i).A_rod-F_mu+Fd; 
end 

