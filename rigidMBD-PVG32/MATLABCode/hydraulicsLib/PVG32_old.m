function [Q_A, Q_B, Q_p] = PVG32_old(U,p_h1, p_h2,p_P, p_t)  

% U: user control signal
% p_h1: hydraulic pressure on piston side
% p_h2: hydraulic pressure on piston-rod side
% p_P: pump pressure
% p_t: tank pressure

param   = Get_parameters_Patu();

if U <= param.u_dl

     % Determine flow rate coefficients and flow rate
     Q_B     = param.Cv_BtoT*(param.u_dl-U)*sqrt(abs(p_h2-p_t))*sign(p_h2-p_t);
     if p_P-p_h1 >= param.p_ref
        % Determine Cv_PtoA
         Q_A     = param.Cv_PtoA*(param.u_dl-U)*sqrt(abs(param.p_ref))*sign(param.p_ref);
     else
        % Determine Cv_PtoA
          Q_A     = param.Cv_PtoA*(param.u_dl-U)*sqrt(abs(p_P-p_h1))*sign(p_P-p_h1);
     end
     Q_p = Q_A;
elseif U >= param.u_dh
     % Determine flow rate coefficients
        Q_A      = -param.Cv_AtoT*(U-param.u_dh)*sqrt(abs(p_h1-p_t))*sign(p_h1-p_t);
     if p_P-p_h2 >= param.p_ref
        % Determine Cv_PtoA
        Q_B     = -param.Cv_PtoB*(U-param.u_dh)*sqrt(abs(param.p_ref))*sign(param.p_ref);
     else
        % Determine Cv_PtoA
        Q_B     = -param.Cv_PtoB*(U-param.u_dh)*sqrt(abs(p_P-p_h2))*sign(p_P-p_h2);
     end

      Q_p = Q_B;
else

    Q_A = 0;
    Q_B = 0;

    Q_p = Q_A;
end