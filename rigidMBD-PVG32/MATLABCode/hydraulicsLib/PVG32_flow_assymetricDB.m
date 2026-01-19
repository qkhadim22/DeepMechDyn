function [Q_A, Q_B, Q_p] = PVG32_flow_assymetricDB(t, U,p_h1, p_h2,p_P,p_t)  


% p_h1, p_h2,p_P,p_t


% U: user control signal
% p_h1: hydraulic pressure on piston side
% p_h2: hydraulic pressure on piston-rod side
% p_P: pump pressure
% p_t: tank pressure

param                      = Get_parameters_Patu();
u                          = (6-U)/3;  %(6-U)/3

if u > param.u0

     Q_A = (u -  param.u0)/(1 -  param.u0)*param.Q0;
      Kv = (u -  param.u0)/(1 -  param.u0)*param.Kv0;

      Q_B = Kv*u*sqrt(abs(p_h2-p_t))*sign(p_h2-p_t);
      Q_p = Q_A;
      t
elseif u > -param.u0
      Q_B = 0;
      Kv = 1e-10;
      Q_A = Kv*u*sqrt(abs(p_h1-p_t))*sign(p_h1-p_t);
      Q_p = Q_B;

      t 

else

     Q_B = -(abs(u) -  param.u0)/(1 -  param.u0)*param.Q0;
     Kv = (abs(u) -  param.u0)/(1 -  param.u0)*param.Kv0;
     Q_A = Kv*u*sqrt(abs(p_h1-p_t))*sign(p_h1-p_t);

     Q_p = Q_A;
end


% if t < param.TMotionExtendingStart
%         pAMeasured = param.pAHold_0;
%         pBMeasured = param.pBHold_0;
% elseif t < param.TMotionExtendingFinish
%         um = (t - param.TMotionExtendingStart)/(param.TMotionExtendingFinish - param.TMotionExtendingStart);
%         pAMeasured = param.pAExtendingStart + um*(param.pAExtendingFinish - param.pAExtendingStart);
%         pBMeasured = param.pBExtendingStart + um*(param.pBExtendingFinish - param.pBExtendingStart);
% elseif t < param.TMotionRetractingStart
%         pAMeasured = param.pAHold_1;
%         pBMeasured = param.pBHold_1;
% elseif t < param.TMotionRetractingFinish
%         um = (t - param.TMotionRetractingStart)/(param.TMotionRetractingFinish - param.TMotionRetractingStart);
%         pAMeasured = param.pARetractingStart + um*(param.pARetractingFinish - param.pARetractingStart);
%         pBMeasured = param.pBRetractingStart + um*(param.pBRetractingFinish - param.pBRetractingStart);
% else
%         pAMeasured = param.pAHold_0;
%         pBMeasured = param.pBHold_0;
 end





