function [F_mu] = FrictionForce_BrownMcPhee(param,dots,i ) 

F_mu  = (param.cyl(i).Fc*tanh(4*(abs(dots)/param.cyl(i).vs))+(param.cyl(i).Fs-param.cyl(i).Fc)*((abs(dots)/param.cyl(i).vs)/.....
                ((1/4)*(abs(dots)/param.cyl(i).vs)^2+3/4)^2))*sign(dots)+param.cyl(i).sig2*dots*tanh(4);
end 