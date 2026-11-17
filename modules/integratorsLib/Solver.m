classdef dynamicSolver < handle
    properties
        TimeStep      double = 1e-3; 
        currentTime   double = 0;
        startingTime  double = 0;
        finalTime     double = 10;
        tolerance     double = 1e-5;
        RelTol        double = 1e-3;
        AbsTol        double = 1e-6; 
        numberofSteps double = 0;

    end
    methods
        %% Assign physical properties
        function Settings = simulationSettings(varargin)
            for i = 1:2:length(varargin)
                name  = varargin{i};
                value = varargin{i+1};

                switch name
                    case 'TimeStep'
                        Settings.TimeStep = value;
                    case 'currentTime'
                        Settings.currentTime = value;
                    case 'startingTime'
                        Settings.startingTime = value;
                    case 'finalTime'
                        Settings.finalTime = value;
                    case 'tolerance'
                        Settings.tolerance = value;
                    case 'RelTol'
                        Settings.RelTol    = value;
                end

            end

            Settings.numberofSteps = ceil(Settings.finalTime/Settings.TimeStep);

        end

        %% Add rigid body
        % function [mbs] = Rk4(fun,y0, mbs)
        % 
        %     dt      = simulationSettings.TimeStep;
        %     tNow    = simulationSettings.currentTime;
        % 
        %     k1      = dt*fun(tNow       , y0);
        %     k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1);
        %     k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2);
        %     k4      = dt*fun(tNow+dt    , y0  +     k3);
        % 
        %     yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
        % 
        % 
        %     % System states
        %     mbs.solver.time = t+dt;
        % end


    end



end
