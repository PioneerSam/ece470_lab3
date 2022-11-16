%% ece470: robot modelling and control
%  lab3: motion planning for the PUMA560 manipulator robot
%  authors: Duo Li, Pranshu Malik, and Varun Sampat
%  date: 12 March 2022

clc;
close all;
clearvars;

%% set up DH Matrix for the PUMA560
% in order of theta_i, d_i, a_i, alpha_i

link_1 = [0 0.76    0       pi/2 ];
link_2 = [0 -0.2365 0.4323  0    ];
link_3 = [0 0       0       pi/2 ];
link_4 = [0 0.4318  0       -pi/2];
link_5 = [0 0       0       pi/2 ];
link_6 = [0 0.20    0       0    ];

DH = [link_1; link_2; link_3; link_4; link_5; link_6];

%% create the PUMA560 robot model

myrobot = mypuma560(DH);

%% test the effect of attractive forces on the robot joints

Hs        = eul2tr([0 pi pi/2]);
Hs(1:3,4) = [-1; 3; 3;]/4;
qs        = inverse(Hs, myrobot);
Hdes        = eul2tr([0 pi -pi/2]);
Hdes(1:3,4) = [3; -1; 2;]/4;
qdes        = inverse(Hdes, myrobot);
tau = att(qs, qdes, myrobot)

%% motion planning without obstacles

qref = motionplan(qs, qdes, 0, 10, myrobot,[], 1e-2);

% visualize results
figure;
t = linspace(0, 10, 300);
q = ppval(qref, t)';
plot(myrobot, q)

%% setup obstacles in environment
% note: we have modified setupobstacle to be in units of [m] so that
% there are no discrepancies when plotting obstacles later on, and more
% distributed changes throughout the lab code for unit conversion are also
% mitigated this way

setupobstacle;

%% torque due to cylindrical obstacle in environment
% note: our lab is in [m] and not [cm]

qtst = 0.9*qs + 0.1*qdes;
tau  = rep(qtst, myrobot, obs{1})

%% torque due to spherical obstacle in environment

qtst = [pi/2 pi 1.2*pi 0 0 0];
tau  = rep(qtst, myrobot, obs{6})

%% motion planning with obstcales in environment

figure;
hold on;
axis([-1 1 -1 1 0 2])
view(-0.32, 0.5)
plotobstacle(obs);
qref = motionplan(qs, qdes, 0, 10, myrobot, obs, 0.01);
t = linspace(0, 10, 300);
q = ppval(qref, t)';
plot(myrobot, q);
hold off;
