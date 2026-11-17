classdef Constraints < handle
    properties
        id           double     = 0;

        % Physical properties
        markerNumbers   struct  = struct([]);
        visualization   struct  = struct([]);
        constrainedAxes double  = [];
        phi           double    = [];  % Position level constraint Eqs
        phi_q         double    = [];  % Constraint jacobian
        dotphi        double    = [];  % Velocity level constraint Eqs
        ddotphi       double    = [];  % Acceleration level constraint Eqs
        I_til         double    = [];
        I             double    = [];

    end

methods

%% Add constraints
function joint = Constraints(varargin)

    for i = 1:2:length(varargin)
        name  = varargin{i};
        value = varargin{i+1};

        switch name
            case 'markerNumbers'
                joint.markerNumbers = value;
            case 'constrainedAxes'
                joint.constrainedAxes = value;
                            % k         = find(joint.constrainedAxes(4:6), 1);

            case 'visualization'
                joint.visualization = value;

                % if isfield(value, 'show') && value.show
                %     joint.visualize();
        end 

     end

    joint.I        = eye(2);
    joint.I_til    = RotationUtils.skew(joint.I(:,2));
   

    % Formulate constraint equations
    joint.phi       = ConstEq_Position(joint);
    joint.phi_q     = Jacobian_ConstEq(joint);
    % joint.dotphi    = ConstEq_Position(joint);
    % joint.ddotphi   = ConstEq_Position(joint);


end

%% Constraint equations
function phi = ConstEq_Position(joint)

    r_i     = positionVec(joint.markerNumbers(1).parentBody,...
                                       joint.markerNumbers(1).localPosition);  
    r_j     = positionVec(joint.markerNumbers(2).parentBody,...
                                joint.markerNumbers(2).localPosition);
    phi     = r_i-r_j;

end 

% function dotphi = ConstEq_Velocity(joint)
%     phi_q        = Jacobian_ConstEq(joint);
%     dotphi       = phi_q*joint.markerNumbers(1).parentBody.velocity;
% end 
% 
% function ddotphi = ConstEq_Acceleration(joint)
% 
%         phi_q     = Jacobian_ConstEq(joint);
%         ddotphi   = phi_q*joint.markerNumbers(1).parentBody.velocity;
% 
% 
% end 

% Jacobian of constraints
function phi_q = Jacobian_ConstEq(joint)

    

    Cq_i = [joint.I  joint.I_til*joint.markerNumbers(1).parentBody.rotMatrix*...
                                joint.markerNumbers(1).localPosition];
    Cq_j = [joint.I  joint.I_til*joint.markerNumbers(2).parentBody.rotMatrix*...
                                joint.markerNumbers(2).localPosition];

   phi_q = [Cq_i -Cq_j];

end 



end






end


