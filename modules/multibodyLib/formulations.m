classdef formulations < handle
    properties
        type        string = ""
    end
    methods 

        function obj = formulation(varargin)
            for i = 1:2:length(varargin)
                name  = varargin{i};
                value = varargin{i+1};

                switch name
                    case 'type'
                        obj.type = value;
                
                end

            end

        end
        %% Assign physical properties
        function [yOut] = semiRecursive(t,y0, mbs)

            yOut = y0;


        end 
    end
end

        


  
