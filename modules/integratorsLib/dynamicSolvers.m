classdef dynamicSolvers < handle
    properties
        type        string = ""
        formulation string = ""
    end
 methods
        %% Assign physical properties
        function Solver = dynamicSolvers (varargin)
            for i = 1:2:length(varargin)
                name  = varargin{i};
                value = varargin{i+1};

                switch name
                    case 'type'
                        Solver.type = value;
                    case 'formulation'
                        Solver.formulation = value;
                      
                end

            end

        end
        %% dynamic formulation
        function fun = getFormulation(dynamicSolver, mbs)
            if strcmpi(dynamicSolver.formulation, 'semiRecursive')
                % mbs,yIn, varargin
                fun = @(t,y)formulations.semiRecursive(t, y, mbs);
            else
                error('dynamicSolvers:UnknownFormulation', '%s formulation has not been included yet.', dynamicSolver.formulation)

            end 
        end 



        %% Solve system dynamics
        function [mbs] = RungeKutta4(dynamicSolver, mbs, simulationSettings)
            
            dt      = simulationSettings.TimeStep;
            tNow    = simulationSettings.currentTime;
            fun     = dynamicSolver.getFormulation(mbs);
            y0      = [0];

            k1      = dt*fun(tNow       , y0, mbs);
            k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1);
            k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2);
            k4      = dt*fun(tNow+dt    , y0  +     k3);

            yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


            % System states
            simulationSettings.currentTime =  tNow+dt;
        end


    end



end
