% returns the torque on joints due to the repulsive potential of one obs; note that
% q and qdes must be row vectors of length equal to number of joints of robot
function tau = rep(q, myrobot, obs)

% compute the current joint poses and distance from obstacle surface
Hqi    = zeros(4, 4, size(q,2)+1);
d_oipi = zeros(3, size(q,2));

Hqi(:,:,1) = trotz(0); % start from robot base frame: default world frame
for i = 1:size(q,2)
    Hqi(:,:,i+1) = forward(q(1:i)', myrobot);
    d_oipi(:,i)  = obs_surf_dist(Hqi(1:3,4,i+1), obs); % dist(oi, pi)
end
    
% nested function to compute vector from obstacle surface to oi
function diff_oipi = obs_surf_dist(oi, obs)
    assert(obs.type == "cyl" || obs.type == "sph");
    
    if obs.type == "cyl"
        oi = oi(1:2);
    end
    
    % get closest point on obstacle surface
    if norm(oi - obs.c) <= obs.R
        psurf = oi;
    else
        psurf = obs.c + (obs.R/norm(oi-obs.c))*(oi-obs.c);
    end
    diff_oipi = oi - psurf;
    
    diff_oipi = oi - psurf;
    if obs.type == "cyl"
        diff_oipi = [diff_oipi; 0];
    end
end

% compute repulsive force
Frep = zeros(3, size(q,2));
for i = 1:size(q,2)
    dist_oipi = norm(d_oipi(:,i));
    if dist_oipi < obs.rho0 % repulsive force only exists till rho0 away
        Frep(:,i) = ((1/dist_oipi - 1/obs.rho0)/(dist_oipi^3))*d_oipi(:,i);
    end
end

% compute torque
tau = zeros(6,1);

for i = 1:size(q,2)
    % compute the geometric translational velocity jacobian at current joint config
    % Jvn = myrobot.jacob0(q, 'trans'); % rtb toolbox inbuilt (gives final jacobian)
    Jvi = jacobvi(q(1:i), Hqi, myrobot);
    tau = tau + ([Jvi zeros(3, size(q,2)-i)]' * Frep(:,i));
end

if norm(tau) ~= 0
    tau = tau/norm(tau);
end

tau = tau';

end