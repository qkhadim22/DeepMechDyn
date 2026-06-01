addpath('../modules/multibodyLib/AddBody/')
addpath('../modules/visualization/')

sys.bodies  = {};
b0          = RigidBody(); 
b1          = RigidBody('position',[0;0;0], ...
                              'mass', 1, 'COM', [0.5;0;0], ...
                              'inertia', [0 0 0;0 0 0; 0 0 0],...
                              'size',[1 ; 0.1; 0.1],...
                              'rotMatrix',RotationUtils.rotZ(pi/4),...
                              'visualization',struct('show', true,'color', [0 0.4470 0.7410]));

sys         = AddBody(sys, b0);
sys         = AddBody(sys, b1);
VisualizeSystem(sys);


% Marker1     = AddMarker(b1, [0 0.5 0]);