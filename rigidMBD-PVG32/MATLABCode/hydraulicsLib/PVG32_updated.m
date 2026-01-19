function [Q_A, Q_B, Q_p] = PVG32_updated(t, U,p_h1, p_h2,p_P,p_t)  


% p_h1, p_h2,p_P,p_t


% U: user control signal
% p_h1: hydraulic pressure on piston side
% p_h2: hydraulic pressure on piston-rod side
% p_P: pump pressure
% p_t: tank pressure

param                      = Get_parameters_Patu();
u                          = (param.U_0-U)/3;  %(6-U)/3


if u > param.u0

     Q_A = (u -  param.u0)/(1 -  param.u0)*param.Q0;
      Kv = (u -  param.u0)/(1 -  param.u0)*param.Kv0;

      Q_B = -Kv*sqrt(abs(p_h2-p_t));
      Q_p = Q_A;

elseif u > -param.u0
      
      Kv = 1e-10;

      Q_A = - Kv*(p_h1-p_t)^0.5 - sign(p_h1 - p_h2)*Kv*sqrt(abs(p_h1 - p_h2));
      Q_B =   sign(p_h1 - p_h2)*Kv*sqrt(abs(p_h1 - p_h2))-Kv*sqrt((p_h2 - p_t));

      Q_p = Q_B;
else

     Q_B = (abs(u) -  param.u0)/(1 -  param.u0)*param.Q0;
     Kv = (abs(u) -  param.u0)/(1 -  param.u0)*param.Kv0;
     Q_A = -Kv*sqrt(abs(p_h1-p_t));

     Q_p = Q_A;
end


 end
