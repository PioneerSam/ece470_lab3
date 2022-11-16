function q = inverse(H, myrobot)

Rd = H(1:3, 1:3); % desired orientation
od = H(1:3,4); % desired position

oc = od - Rd*[0; 0; myrobot.d(6)];

xc = oc(1);
yc = oc(2);
zc = oc(3);

%% calculate the required length to solve for theta 1,2,3 geometrically 
RT = zc - myrobot.d(1);
PT = sqrt(xc^2 + yc^2 - myrobot.d(2)^2);
PR = sqrt((zc - myrobot.d(1))^2 + xc^2 + yc^2 - myrobot.d(2)^2);
PQ = myrobot.a(2);

% theta 1: 
theta_1 = atan2(yc, xc) - atan2(-myrobot.d(2), PT);

%theta 3: 
D = (myrobot.d(2)^2 + myrobot.d(4)^2 + myrobot.a(2)^2 + xc^2 + yc^2 - (zc - myrobot.d(1))^2) ...
    /(2*myrobot.d(4)*myrobot.a(2));
theta_3 = atan2(D, real(sqrt(1-D^2)));

%theta 2: 
theta_2 = atan2(zc-myrobot.d(1), PT) - ...
    atan2(-myrobot.d(4)*cos(theta_3), myrobot.a(2) + myrobot.d(4)*sin(theta_3));

%% kinematic decoupling: 

% calculate the position first: 
q = [theta_1 theta_2 theta_3]; 

H_0_3 = forward(q', myrobot);  
R_0_3 = H_0_3(1:3, 1:3);
R_desired = H(1:3, 1:3); 
% rotational matrix = R_0_3' * R_desired -- rotation matrix at the wrist
% joint 
R_3_6 = R_0_3' * R_desired; 

%wrist angles theta 4, 5, 6
theta_4 = atan2(R_3_6(2,3), R_3_6(1,3));
theta_5 = atan2(real(sqrt(1- R_3_6(3,3)^2)), R_3_6(3,3));
theta_6 = atan2(R_3_6(3,2), -R_3_6(3,1));

q = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]; % append the wrist angles

end