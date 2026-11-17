classdef simulationSettings < handle
    properties
        TimeStep      double = 1e-3; 
        currentTime   double = 0;
        startingTime  double = 0;
        finalTime     double = 10;
        tolerance     double = 1e-5;
        RelTol        double = 1e-3;
        AbsTol        double = 1e-6; 
        numberofSteps double = 0;
        Time          double = [];
       
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
         
           Settings.numberofSteps = ceil(Settings.finalTime/Settings.TimeStep)+1;

        end

end



end
