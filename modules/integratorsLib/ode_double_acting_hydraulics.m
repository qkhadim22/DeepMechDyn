function y = ode_double_acting_hydraulics(t,x,optionalVec, param)

%x(1):Actuator position->s
%x(2):Actuator veloity->dots
%x(3):Piston side pressure->ph1
%x(4):Piston-rod side pressure->ph1
%x(5):Spool position->U

s     = x(1);
ds    = x(2);
pA    = x(3); 
pB    = x(4); 
U     = x(5);
At    = 1e-3;

Uref  = optionalVec(1);
p_P   = optionalVec(2);
p_t   = optionalVec(3);

dU     = (Uref-U)/param.tau;
Uk     = Uref+At*dU;

% Friction force
F_mu    = FrictionForce_BrownMcPhee(param,ds );
FfrMax  = 0;

vEps = 1e-5;
if ds > vEps
        Ffr = FfrMax;
else
        Ffr = -FfrMax;
end

Fh    = pA*param.A_cyl-pB*param.A_rod-F_mu-Ffr;
netF  = Fh-param.m*param.g;

% Derivative of x = y
[pA_dot,pB_dot] = pressure_derivatives(t,param, s,ds,pA,pB,Uk ,p_P,p_t);


y(1)   = ds;          % ODE 1
y(2)   = netF/param.m;  % ODE 2
y(3)   = pA_dot;
y(4)   = pB_dot;
y(5)   = dU;

y      = y(:);
end 