classdef EffectiveBulkModulus < handle

    % param: Hydraulic parameters
    %     s: Actuator position
    %    pA: Pressure of piston side
    %    pB: Pressure on piston-rod side
    % Be_h1: Effective Bulk modulus on piston side
    % Be_h2: Effective Bulk modulus on piston-rode side


    methods (Static)
        
        %% Nominal bulk modulus model
        function [Be_h1,Be_h2] = Nominal(param,s)

            Vh1 = param.VA;
            Vh2 = param.VB;
            Bh  = param.Beh;
            Bc  = param.Bec;
            Bo  = param.BeO;

            l_A = s - param.cyl.l_cyl;
            l_B = param.cyl.l_rod - l_A;
            V_A = Vh1+param.cyl.A_cyl*l_A;
            V_B = Vh2+param.cyl.A_rod*l_B;

          Be_h1 = 1 / ((1/Bo) + ...
                (param.cyl.A_cyl*l_A/(V_A*Bc)) + ...
                (Vh1/(V_A*Bh)) );

          Be_h2 = 1 / ((1/Bo) + ...
                (param.cyl.A_rod*l_B/(V_B*Bc)) + ...
                (Vh2/(V_B*Bh)) );

        end

        
        %% PressureBased bulk modulus model
        function [Be_h1,Be_h2] = PressureBased(param,pA,pB)

            uBeta = pA/param.pBetaA;

            if uBeta > 1
                Be_h1 = param.betaMax;
            else
                Be_h1 = (3*uBeta^2 - 2*uBeta^3)*param.betaMax;
            end

            uBeta = pB/param.pBetaB;

            if uBeta > 1
                Be_h2 = param.betaMax;
            else
                Be_h2 = (3*uBeta^2 - 2*uBeta^3)*param.betaMax;
            end

        end
 
        %% Wylie bulk modulus model
        function [Be_h1,Be_h2] = Wylie(param,pA,pB)

            P0      = param.P0;
            lambdaA = param.lambdaA;
            lambdaB = param.lambdaA;
            alphaA  = param.cyl(1).alphaA;
            alphaB  = param.cyl(1).alphaB;

            cA      = alphaA * (P0/pA)^(1/lambdaA) * (param.BeO/(lambdaA*pA) - 1);
            cB      = alphaB * (P0/pB)^(1/lambdaB) * (param.BeO/(lambdaB*pB) - 1);
            Be_h1   = param.BeO/(1 + cA);
            Be_h2   = param.BeO/(1 + cB);

        end
        
        %% Nykanen bulk modulus model
        function [Be_h1,Be_h2] = Nykanen(param,pA,pB)

            P0A     = param.P0;
            P0B     = param.P0;
            lambdaA = param.lambdaA;
            lambdaB = param.lambdaB;
            alphaA  = param.cyl(1).alphaA;
            alphaB  = param.cyl(1).alphaB;

            cA      = alphaA .* (P0A ./ pA).^(1./lambdaA);
            dA      = (1-alphaA) ./ (1 + (pA-P0A)./param.BeO);
            cB      = alphaB .* (P0B ./ pB).^(1./lambdaB);
            dB      = (1-alphaB) ./ (1 + (pB-P0B)./param.BeO);

            Be_h1   = (cA+dA) ./  ( cA./(lambda.*pA) + ...
                                (1-alphaA)./((1+(pA-P0A)./param.BeO).^2 .* param.BeO) );

            Be_h2   = (cB+dB) ./ ( cB./(lambda.*pB) + ...
                            (1-alphaB)./((1+(pB-P0B)./param.BeO).^2 .* param.BeO) );

        end

        %% Ruan bulk modulus model
        function [Be_h1,Be_h2] = Ruan(param,pA,pB)

            P0      = param.P0;
            alphaA  = param.cyl(1).alphaA;
            alphaB  = param.cyl(1).alphaB;
            pcA     = param.pcA;
            pcB     = param.pcB;

            if (pA > P0) && (pA < pcA)
                Be_h1 = param.BeO / (alphaA*P0*param.BeO*(1/pA^2 - 1/pcA^2) + 1);
            else
                Be_h1 = param.BeO;
            end

            if (pB > P0) && (pB < pcB)
                Be_h2 = param.BeO /(alphaB*P0*param.BeO*(1/pB^2 - 1/pcB^2) + 1);
            else
                Be_h2 = param.BeO;
            end

        end

    end

end