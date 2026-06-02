classdef RigidBody < handle
    properties
        id

        % Physical properties
        size        = [0 0 0]';
        mass        = [];
        COM         = [0 0 0]';
        inertia     = zeros(3,3);

        % Graphics
      visualization = struct([]);
      graphics      = struct([]);

        % Markers
        markers     = struct([]);

        % Configuration
        position    = [0 0 0]';
        local_offset= [0 0 0]';

        rotation    = [0 0 0]';
        rotMatrix   = eye(3);

        % Velocities
        velocity           = [0 0 0]';
        omega              = [0 0 0]';
        drotMatrix         = eye(3);


        % Acceleration
        acceleration       = [0 0 0]'; 
        alpha              = [0 0 0]'; 
        ddrotMatrix        = eye(3);

        % External forces
        force        = [0 0 0]';
        moment       = [0 0 0]';
    end

methods

%% Assign physical properties
function body = RigidBody(varargin)
            % Constructor (replaces CreateRigidBody)

    for i = 1:2:length(varargin)
        name  = varargin{i};
        value = varargin{i+1};

        switch name
            case 'mass'
                body.mass = value;
            case 'COM'
                body.COM = value;
            case 'position'
                body.position = value;
            case 'inertia'
                 if isempty(body.inertia)
                     body.inertia = (body.mass/12)*diag([body.size(2)^2 + body.size(3)^2, ...
                                                        body.size(1)^2 + body.size(3)^2, ...
                                                        body.size(1)^2 + body.size(2)^2]);
                 else
                     body.inertia = value;
                 end

            case 'size'
                body.size = value;
            case 'rotMatrix'
                body.rotMatrix = value;
            case 'visualization'
                body.visualization = value;

                if isfield(value, 'show') && value.show
                    body.visualize();
                end 
        end
    end
end

%% Position
function rj = positionVec(body, uVec )
    rj      = body.position(:)+body.rotMatrix*uVec(:);
end 

%% Velocity
function dotrj = velocityVec(body, uVec )
     dotrj     = body.velocity(:)+cross(body.omega, uVec);
end 

%% Acceleration
function ddotrj = accVec(body, uVec )
    ddotrj      = body.acceleration(:)+...
                        cross(body.alpha, uVec(:))+...
                         cross(body.omega, cross(body.omega, uVec(:))) ;
end 

%% AddMarker
function Marker = AddMarker(body, varargin)

    % store in body
    if isempty(body.markers)
        Marker.id = 1;
    else
        Marker.id = length(body.markers) + 1;
    end


    for i = 1:2:length(varargin)
        name  = varargin{i};
        value = varargin{i+1};

        switch name
            case 'localPosition'
                Marker.localPosition = value;
                if isempty(body.markers)
                    body.markers = Marker;
                else
                    body.markers(end+1) = Marker;
                end

            case 'visualization'
                Marker.visualization = value;

                if isfield(value, 'show') && value.show
                    body.visualize()
                end
        end
    end

   

end


%% Assign graphics
function g = plot_body(body)

    p0      = body.positionVec(body.local_offset(:));
    pend    = body.positionVec([body.size(1);0;0]);

    core_line   = [p0  pend]';

    poly        = polybuffer(core_line(:,1:2), 'lines', body.size(2)/2, 'JointType', 'round');

    g.type      = 'bar2D';
    g.vertices  = poly.Vertices;

end

% Draw local point on the body
function   drawLocalPoint(body, uVec, axis_len,ax)

    if nargin < 4 || isempty(ax), ax = gca; end
    if isempty(axis_len) || ~isnumeric(axis_len) || ~isscalar(axis_len)
        axis_len = 0.1;
    end

    p       = body.positionVec(uVec(:));
    local_x = body.rotMatrix(1:2, 1)*axis_len;
    local_y = body.rotMatrix(1:2, 2)*axis_len;
    
    quiver(ax,p(1), p(2), local_x(1), local_x(2), 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver(ax,p(1), p(2), local_y(1), local_y(2), 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

    plot(ax,p(1), p(2), 'o', ...
        'MarkerFaceColor', 'k', ...
        'MarkerEdgeColor', 'k');
end

% Visualize body
function visualize(body, ax)

    if isfield(body.visualization,'color')
        c = body.visualization.color;
    else
        c = [0 0.4470 0.7410];
    end

    g = body.plot_body();
    v = g.vertices;

    axis(ax, [-2*max(body.size) 2*max(body.size) ....
        -2*max(body.size) 2*max(body.size)]);

    hold(ax, 'on');


    fill(ax, v(:,1), v(:,2), c, 'FaceAlpha', 0.5);
    axis(ax, 'equal');
    grid(ax, 'on');

    p0 = body.positionVec(body.local_offset);
    axis_len = 0.15 * max(body.size);

    ex = body.rotMatrix(1:2,1)*axis_len;
    ey = body.rotMatrix(1:2,2)*axis_len;

    quiver(ax,p0(1), p0(2), ex(1), ex(2), 0, 'r');
    quiver(ax,p0(1), p0(2), ey(1), ey(2), 0, 'g');

    body.drawLocalPoint(body.local_offset,axis_len,ax); 
    body.drawLocalPoint(body.COM,axis_len,ax);         

    if  ~isempty(body.markers)
        for i = 1:length(body.markers)
            m = body.markers(i);
            body.drawLocalPoint(m.localPosition, axis_len,ax);
        end
    end 

    hold off;
    struct('type','bar2D','vertices',v);

end

end


end


