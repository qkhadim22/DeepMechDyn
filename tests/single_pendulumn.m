addpath('../modules/multibodyLib/')
addpath('../modules/visualization/')

mbs         = MultibodySystem();

% Ground body
b0          = mbs.addRigidBody(); 

% Body 1
b1          = mbs.addRigidBody('position',[0;0;0], ...
                              'mass', 1, 'COM', [0.5;0;0], ...
                              'inertia', [0 0 0;0 0 0; 0 0 0],...
                              'size',[1 ; 0.1; 0.1],...
                              'rotMatrix',RotationUtils.rotZ(pi/4),...
                              'visualization',struct('False', true,'color', [0 0.4470 0.7410]));
Marker1     = b1.AddMarker('localPosition' , [1;0;0], ...
                           'visualization', struct('show', false) );


mbs.Assemble()
mbs.Visualize()
