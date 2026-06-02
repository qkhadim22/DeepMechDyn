classdef MultibodySystem < handle


    properties
        bodies = {};
        joints = {};
    end
methods


%% Add rigid body
function body = addRigidBody(mbs, varargin)

        body                = RigidBody(varargin{:});
        mbs.bodies{end+1}   = body;
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
