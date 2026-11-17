addpath('../modules/multibodyLib/')
addpath('../modules/visualization/')
addpath('../modules/integratorsLib/')

%% Solver settings
mbs             = MultibodySystem();
simulation      = simulationSettings('TimeStep', 5e-3,'finalTime', 1);

%% Model a system

% Define bodies
% Ground body
b0              = mbs.addRigidBody(); 
Marker0         = b0.AddMarker('localPosition' , [0;0], ...
                            'visualization', struct('show', false) );

% Body 1
b1              = mbs.addRigidBody('position',[0;0], ...
                              'mass', 1, 'COM', [0.5;0], ...
                              'inertia', [0 0 0;0 0 0; 0 0 0],...
                              'size',[1 ; 0.1; 0.1],...
                              'rotMatrix',RotationUtils.rotZ(pi/4),...
                              'visualization',struct('False', true,'color', [0 0.4470 0.7410]));
Marker1         = b1.AddMarker('localPosition' , [0;0], ...
                           'visualization', struct('show', false) );


% Add bodies
mbs.addJoint('markerNumbers', [Marker0, Marker1],...
              'constrainedAxes', [0,0,0,0,0,1]   );


%% Dynamic solver 

dynamicSolver    = dynamicSolvers('formulation', 'semiRecursive');

for i = 1:simulation.numberofSteps
simulation.Time(i)      = simulation.currentTime;
[mbs]                   = dynamicSolver.RungeKutta4(mbs, simulation);


end 




mbs.Visualize()
