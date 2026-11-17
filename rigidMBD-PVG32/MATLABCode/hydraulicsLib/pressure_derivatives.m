function [pA_dot,pB_dot,Be_h1,Be_h2,u] = pressure_derivatives(t,param, s,ds,pA,pB,U,i)

switch param.valve_models
    case 'PVG32_flow_assymetricDB'
        [Q_A,Q_B,~, u] = PVG32_flow_assymetricDB(param, U,pA,pB) ;
    case 'PVG32_old'
        [Q_A,Q_B,~] = PVG32_old(param, U,pA,pB) ;
        u           = U; 
    otherwise
        [Q_A,Q_B,~] = PVG32_laminar(param, U,pA,pB) ;
        u           = U; 
end

l_A         = s - param.cyl(i).l_cyl;
l_B         = param.cyl(i).l_rod - l_A;
V_A         = param.VA+param.cyl(i).A_cyl*l_A;
V_B         = param.VB+param.cyl(i).A_rod*l_B;

% if (pA < 1e5) || (pB < 1e5)
if i<2 
 switch param.bulkType
    case 'Nominal'
        [Be_h1,Be_h2] = EffectiveBulkModulus(param,pA,pB,l_A,l_B,V_A,V_B);

    case 'Nykanen'
        [Be_h1,Be_h2] = Nykanen(param,pA,pB);

    case 'Ruan'
        [Be_h1,Be_h2] = Ruan(param,pA,pB,V_A,V_B);

    case 'Wylie'
        [Be_h1,Be_h2] = Wylie(param,pA,pB);

    case 'Pressures'
        [Be_h1,Be_h2] = EffectiveBulkModulus_Pressures(param,pA,pB);

    otherwise
        error('Unknown BulkModulus type');
end

else 
    [Be_h1,Be_h2] = EffectiveBulkModulus(param,pA,pB, l_A,l_B,V_A,V_B);
end

% % Calculate the pressure derivatives
pA_dot        = (Be_h1/V_A)*(Q_A-param.cyl(i).A_cyl*ds);
pB_dot        = (Be_h2/V_B)*(param.cyl(i).A_rod*ds-Q_B);

end


function [Be1,Be2] = EffectiveBulkModulus(param,pA,pB,l_A, l_B, V_A, V_B) 

Vh1     = param.VA;
Vh2     = param.VB;

Bh      = param.Beh;
Bc      = param.Bec;
Bo      = param.BeO;
B_air1  = 1.4*pA;
B_air2  = 1.4*pB;

Be1 = 1/((1/Bo)+(param.cyl(1).A_cyl*l_A/(V_A*Bc))+(Vh1/(V_A*Bh))   ); 

Be2 = 1/((1/Bo)+(param.cyl(2).A_rod*l_B/(V_B*Bc))+(Vh2/(V_B*Bh)) );

end



function [Be_h1,Be_h2] = EffectiveBulkModulus_Pressures(param,pA, pB) 

    uBeta       = pA/param.pBetaA;
    if uBeta > 1
        Be_h1 = param.betaMaxA;
    else
        Be_h1 = (3*uBeta^2 - 2*uBeta^3)*param.betaMaxA;
    end

    uBeta = pB/param.pBetaB;
    if uBeta > 1
        Be_h2 = param.betaMaxB;
    else
        Be_h2 = (3*uBeta^2 - 2*uBeta^3)*param.betaMaxB;
    end

end 



function [Be_h1,Be_h2] = Wylie(param,pA, pB) 

    P0       = 1e5;
    lambdaA  = 1.4;

    lambdaB  =  1.4;
    alphaA   =  param.cyl(1).alphaA;
    alphaB   =  param.cyl(1).alphaB;
    cA       =  alphaA * (P0 / pA)^(1/lambdaA)* (param.BeO/ (lambdaA* pA) - 1);
    cB       =  alphaB * (P0 / pB)^(1/lambdaB)* (param.BeO/ (lambdaB* pB) - 1);

    Be_h1 = param.BeO ./ (1 + cA);

    Be_h2 = param.BeO ./ (1 + cB);


end 


function [Be_h1,Be_h2] = Nykanen(param,pA, pB) 


   P0B       = 1e5;
    P0A      = P0B;
    lambda   = 1.4;
    alphaA   =  param.cyl(1).alphaA;
    alphaB   =  param.cyl(1).alphaB;


     cA     = alphaA .* (P0A ./ pA).^(1./lambda);
     dA     = (1 - alphaA) ./ (1 + (pA - P0A)./param.BeO);

     cB     = alphaB .* (P0B ./ pB).^(1./lambda);
     dB     = (1 - alphaB) ./ (1 + (pB - P0B)./param.BeO);


    Be_h1 = (cA + dA) ./ ( cA./(lambda .* pA) + ...
                    (1 - alphaA) ./ ( (1 + (pA - P0A)./param.BeO).^2 .* param.BeO ) );


    Be_h2 = (cB + dB) ./ ( cB./(lambda .* pB) + ...
                    (1 - alphaB) ./ ( (1 + (pB - P0B)./param.BeO).^2 .* param.BeO ) ); 


end 


function [Be_h1,Be_h2] = Ruan(param,pA, pB,V_A,V_B) 


    P0      = 1e5;
    alphaA   =  param.cyl(1).alphaA;
    alphaB   =  param.cyl(1).alphaB;

    pcA      = param.pCA;
    pcB      = param.pCB;


if (pA > P0) && (pA < pcA)
    Be_h1 = param.BeO ./ ( alphaA * P0 * param.BeO * (1./pA^2 - 1./pcA^2) + 1 );
else
    Be_h1 = param.BeO;
end

% if (pB > P0) && (pB < pcB)
   Be_h2 = param.BeO ./ ( alphaB * P0 * param.BeO * (1./pB^2 - 1./pcB^2) + 1 );
% else
%     Be_h2 = param.BeO;
% end 


end 