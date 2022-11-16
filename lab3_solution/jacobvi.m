% returns the geometric velocity jacobian treating current link as the end-effector
function Jvi = jacobvi(q, Hqi, myrobot)

% initialize effective number of joints
n   = size(q,2);
Jvi = zeros(3,n);

for i = 1:size(q,2)
    Jvoi = Hqi(1:3,3,i); % zoi
    if isrevolute(myrobot.links(i))
        % note: Hqi starts from base, thus end-effector index is n+1 
        Jvoi = cross(Jvoi, Hqi(1:3,4,n+1)-Hqi(1:3,4,i)); % zoi x (on - oi)
    end
    Jvi(:,i) = Jvoi;
end

end

