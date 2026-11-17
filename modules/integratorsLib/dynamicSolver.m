classdef dynamicSolver < handle
    properties
        type        string = ""
        formulation string = ""
     simulationTime double = [0];


    end
 methods
        %% Assign physical properties
        function Solver = dynamicSolver(mbs, Settings, varargin)
            for i = 1:2:length(varargin)
                name  = varargin{i};
                value = varargin{i+1};

                switch name
                    case 'simulationTime'
                          Solver.simulationTime = simulationTime;
                    case 'type'
                        Solver.type = value;

                        if strcmpi(Solver.type, "RK4")
                            [mbs, Settings] = Solver.RungeKutta4(mbs, Settings);
                        end
                    case 'formulation'
                        Solver.formulation = value;
                      
                end

            end


        end

        %% Solve system dynamics
        function [mbs,dynamicSolver] = RungeKutta4(Solver, mbs, simulationSettings, varargin)
            
            dt                             = simulationSettings.TimeStep;
            tNow                           = simulationSettings.currentTime;
            
            % if strcmpi(Solver.formulation, 'semiRecursive')
            %       fun = Solver.formulation();
            % end 
            % 
            % k1      = dt*fun(tNow       , y0);
            % k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1);
            % k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2);
            % k4      = dt*fun(tNow+dt    , y0  +     k3);
            % 
            % yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


            % System states
            dynamicSolver.currentTime = tNow+dt;
        end


    end



end
