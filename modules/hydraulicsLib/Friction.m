classdef Friction < handle

    % param: Hydraulic parameters
    %  dots: Actuator velocity
    % F_mu: Hydraulic friction force

    methods (Static)
        
        %% 4/3 directional control valve
        function [F_mu] = BrownMcPhee(param,dots) 

           F_mu  = (param.cyl.Fc*tanh(4*(abs(dots)/param.cyl.vs))+(param.cyl.Fs-param.cyl.Fc)*((abs(dots)/param.cyl.vs)/.....
                        ((1/4)*(abs(dots)/param.cyl.vs)^2+3/4)^2))*sign(dots)+param.cyl.sig2*dots*tanh(4);
        end

    end

end