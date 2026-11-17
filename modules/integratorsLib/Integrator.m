classdef Integrator < handle
    properties
        id
        bodies = {};
        joints = {};
    end
    methods
        
%% Add rigid body
function state = Rk4(fun,tNow,y0, simulationSettings)

dt      = simulationSettings.TimeStep;

k1      = dt*fun(tNow       , y0);
k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1);
k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2);
k4      = dt*fun(tNow+dt    , y0  +     k3);

yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
end

        %% Add joint
        function joint = addJoint(mbs,varargin)

            joint  = Constraints(varargin{:});

            if isempty(mbs.joints)
                ID = 1;
            else
                ID = length(mbs.joints) + 1;
            end
            joint.id        = ID;
            mbs.joints{end+1}   = joint;
        end

        %% Build a multibody system
        function sys = Assemble(mbs)

            for j = 1:numel(mbs.joints)

                sys = mbs.joints{j};
            end 

        end

        %% Visualize the system
        function Visualize(mbs)
            if isempty(mbs.bodies)
                error('Multibody system does not contain any body.');
            end

            fig = figure;
            ax  = axes(fig);
            hold(ax,'on');
            axis(ax,'equal');
            grid(ax,'on');


            % Plot bodies
            for i = 1:numel(mbs.bodies)
                if isempty(fieldnames(mbs.bodies{i}.visualization))
                    continue;
                end
                mbs.bodies{i}.visualize(ax);  
            end

            hold(ax,'off');
        end

    end



end
