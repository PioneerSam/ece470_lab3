% returns the torque on joints due to the attractive potential; note that
% q and qdes must be row vectors of length equal to number of joints of robot
function tau = att(q, qdes, myrobot)

% compute the distance between the current and desired joint positions
Hqi    = zeros(4, 4, size(q,2)+1);
oqdiff = zeros(3, size(q,2));

Hqi(:,:,1) = trotz(0); % start from robot base frame: default world frame
for i = 1:size(q,2)
    Hqi(:,:,i+1) = forward(q(1:i)', myrobot);
    Hqdesi       = forward(qdes(1:i)', myrobot);
    oqdiff(:,i)  = Hqi(1:3,4,i+1) - Hqdesi(1:3,4);
end

% compute attractive force
d    = 1.0;     % attraction saturation distance (m)
Fatt = -oqdiff; % force is along negative gradient of potential field

for i = 1:size(oqdiff,2)
     if norm(oqdiff(i)) >= d
        Fatt(i) = Fatt(i) * d/norm(oqdiff(i)); % normalize (saturate)
    end
end

% compute torque
tau = zeros(6,1);

for i = 1:size(q,2)
    % compute the geometric translational velocity jacobian at current joint config
    % Jvn = myrobot.jacob0(q, 'trans'); % rtb toolbox inbuilt (gives final jacobian)
    Jvi = jacobvi(q(1:i), Hqi, myrobot);
    tau = tau + ([Jvi zeros(3, size(q,2)-i)]' * Fatt(:,i));
end

if norm(tau) ~= 0
    tau = tau/norm(tau);
end

tau = tau';

end