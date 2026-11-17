classdef Valves < handle

    % param: Hydraulic parameters
    %     U: User control signal
    %    pA: Pressure of piston side
    %    pB: Pressure on piston-rod side
    %   p_P: Pump pressure
    %   p_t: Tank pressure

    % Q_A: Flow-rate on piston side
    % Q_B: Flow-rate on piston-rode side

    methods (Static)
        
        %% 4/3 directional control valve
        function [Q_A, Q_B] = DCV43(param, U,pA, pB,p_P,p_t) 

            if U >= 1e-6
                Q_A = param.CvA*(U)*sqrt(abs(p_P-pA))*sign(p_P-pA);
                Q_B = param.CvA*(U)*sqrt(abs(pB-p_t))*sign(pB-p_t);

            elseif -1e-6 < U && U < 1e-6 % U == 0,
                Q_A = 0;
                Q_B = 0;

            else
                Q_A = param.CvA*(U)*sqrt(abs(pA-p_t))*sign(pA-p_t);
                Q_B = param.CvA*(U)*sqrt(abs(p_P-pB))*sign(p_P-pB);
            end

        end

        
        %% Novel PVG32 valve model
        function [Q_A, Q_B] = PVG32_assymetricDB(param, U,pA, pB,p_t)
            
            u  = (param.valve(1).U_N -U)/3; 
            
            if u > param.valve(1).u0
            
                 Q_A    = ((u -  param.valve(1).u0)/(1 -  param.valve(1).u0))*param.valve(1).Q0;

                  Kv    = ((u -  param.valve(1).u0)/(1 -  param.valve(1).u0))*param.valve(1).Kv0;
                  Q_B   = Kv*sqrt(abs(pB-p_t))*sign(pB-p_t);
                 
                  % Q_p = Q_A;
            elseif u > -param.valve(1).u0
                  Q_B   = 1e-10;
                  Q_A   = 0;

                  %Kv = 1e-10;
                  % Q_A = Kv*u*sqrt(abs(p_h1-param.p_t))*sign(p_h1-param.p_t);
                  % Q_p = Q_B;
            else
                 Q_B    = -(abs(u) -  param.valve(1).u0)/(1 -  param.valve(1).u0)*param.valve(1).Q1;
                 
                 Kv     =   (abs(u) -  param.valve(1).u0)/(1 -  param.valve(1).u0)*param.valve(1).Kv1;
                 Q_A    = - Kv*sqrt(abs(pA-p_t))*sign(pA-p_t);
                 % Q_p = Q_A;
            end

        end
 
        %% PVG32 old valve model
        function [Q_A, Q_B] = PVG32_Nominal(param, U,pA, pB,p_P, p_t)
            
            
            if U <= param.u_dl
                 
                 Q_B     = param.Cv_BtoT*(param.u_dl-U)*sqrt(abs(pB-p_t))*sign(pB-p_t);
                 
                 if p_P-p_h1 >= param.p_ref
                     Q_A     = param.Cv_PtoA*(param.u_dl-U)*sqrt(abs(param.p_ref))*sign(param.p_ref);
                 else
                    % Determine Cv_PtoA
                      Q_A     = param.Cv_PtoA*(param.u_dl-U)*sqrt(abs(p_P-pA))*sign(p_P-pA);
                 end
                 % Q_p = Q_A;

            elseif U >= param.u_dh
                    Q_A      = -param.Cv_AtoT*(U-param.u_dh)*sqrt(abs(pA-p_t))*sign(pA-p_t);
                 if p_P-p_h2 >= param.p_ref
                    Q_B     = -param.Cv_PtoB*(U-param.u_dh)*sqrt(abs(param.p_ref))*sign(param.p_ref);
                 else
                    Q_B     = -param.Cv_PtoB*(U-param.u_dh)*sqrt(abs(p_P-pB))*sign(p_P-pB);
                 end
                % Q_p = Q_B;
            else
                Q_A = 0;
                Q_B = 0;
                % Q_p = Q_A;
            end

        end

       %% Valve leakage ___ Under development
       
       % function [Q_A, Q_B] = ValveLeakage(param, U,pA, pB,p_P,p_t) 
       % 
       %     if U >= 1e-6
       %         Q_A = param.CvA*(U)*sqrt(abs(p_P-pA))*sign(p_P-pA);
       %         Q_B = param.CvA*(U)*sqrt(abs(pB-p_t))*sign(pB-p_t);
       % 
       %     elseif -1e-6 < U && U < 1e-6 % U == 0,
       %         Q_A = 0;
       %         Q_B = 0;
       % 
       %     else
       %         Q_A = param.CvA*(U)*sqrt(abs(pA-p_t))*sign(pA-p_t);
       %         Q_B = param.CvA*(U)*sqrt(abs(p_P-pB))*sign(p_P-pB);
       %     end
       % 
       % end




    end

end