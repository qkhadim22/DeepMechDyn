classdef MultibodySystem < handle
    properties
        id      double  = 0;       % IDs for the bodies and joints 
        bodies  cell    = {};      % Cell array of bodies
        joints  cell    = {};      % Cell array of joints
    end
methods


%% Add rigid body
function body = addRigidBody(mbs, varargin)
    
    body     = RigidBody(varargin{:});
    if isempty(mbs.bodies)
        ID = 1;
    else
        ID = length(mbs.bodies) + 1;
    end

    body.id             = ID;
    mbs.bodies{end+1}   = body;
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
