function [Q_A, Q_B, Q_p, u] = PVG32_laminar(param, U,p_h1, p_h2)  

% p_h1, p_h2,p_P,param.p_t


% U: user control signal
% p_h1: hydraulic pressure on piston side
% p_h2: hydraulic pressure on piston-rod side
% p_P: pump pressure
% param.p_t: tank pressure

u                          = (param.valve(1).U_N -U)/3;  %(6-U)/3,5.8

if u > param.valve(1).u0

    % Qin = (u - u0)/(1 - u0)*Q0;
    %    Kv = (u - u0)/(1 - u0)*Kv0;

    Q_A = ((u -  param.valve(1).u0)/(1 -  param.valve(1).u0))*param.valve(1).Q0;
    Kv = ((u -  param.valve(1).u0)/(1 -  param.valve(1).u0))*param.valve(1).Kv0;
    
    dp   = p_h2 - param.p_t;
    p_tr = 5e5;                                   % transition pressure [Pa]
    Cv   =param.Kv ;
    if abs(dp) <= p_tr
        Q_B = Cv*dp;                   % laminar: linear, same value as turbulent at |dp| = p_tr
    else
        Q_B = Kv*sqrt(abs(dp))*sign(dp);          % turbulent
    end

    Q_p = Q_A;
elseif u > -param.valve(1).u0
    Q_B = 1e-10;
    %Kv = 1e-10;
    % Q_A = Kv*u*sqrt(abs(p_h1-param.p_t))*sign(p_h1-param.p_t);
    Q_p = Q_B;
    Q_A = 0;
else
    % Qin = -(abs(u) - u0)/(1 - u0)*Q0;
    %        Kv = (abs(u) - u0)/(1 - u0)*Kv0;
    %        Qin_ALT = -Kv*(pAMeasured*1e5)^0.5;

    dp   = p_h2 - param.p_t;
    p_tr = 7e5;                                   % transition pressure [Pa]
    kv0  = (abs(u) -  param.valve(1).u0)/(1 -  param.valve(1).u0); 
    Cv   = param.Kv;
    if abs(dp) <= p_tr
        Q_B = Cv*dp;                   % laminar: linear, same value as turbulent at |dp| = p_tr
    else
        Q_B = -kv0*param.valve(1).Q1;
    end

    Kv = (abs(u) -  param.valve(1).u0)/(1 -  param.valve(1).u0)*param.valve(1).Kv1;
    Q_A = -Kv*sqrt(abs(p_h1-param.p_t))*sign(p_h1-param.p_t);

    Q_p = Q_B;
end

