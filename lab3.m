clc;
close all;
clear;

%% define the robot structure 

% initialize all thetas to zero
link_1 = [0 0.76    0       pi/2    ];
link_2 = [0 -0.2365 0.4323  0       ];
link_3 = [0 0       0       pi/2    ];
link_4 = [0 0.4318  0       -pi/2   ];
link_5 = [0 0       0       pi/2    ];
link_6 = [0 0.20    0       0       ];

% the DH table: 
DH = [link_1; link_2; link_3; link_4; link_5; link_6];

% construct the robot using the toolbox function 'Link' and 'SerialLink'
myrobot = mypuma560(DH);






%% Test
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot);
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
% This is the final joint variable vector
tau = att(q1,q2,myrobot)
