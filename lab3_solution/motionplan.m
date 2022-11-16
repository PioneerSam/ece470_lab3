% runs gradient descent while end-effector reaches goal within a tolerance
function qref = motionplan(qs, qdes, t1, t2, myrobot, obs, tol)

% define hyperparameters
alpha = 0.01; % ~ step size
eta   = 0.92; % scaling factor for repulsive force

% perform gradient descent over environment with obstacles
q       = zeros(1,6);
q(1, :) = qs;
while norm(wrapTo2Pi(q(end, 1:5)) - wrapTo2Pi(qdes(1:5))) > tol % stop condition
    
    % get total repulsion effect from all obstacles
    tau = zeros(1, 6);
    for i = 1:size(obs, 2)
        tau = tau + rep(q(end, 1:6), myrobot, obs{i});
    end
    
    % add attraction effect of the goal pose
    tau = eta*tau + att(q(end, 1:6), qdes, myrobot);
    
    qprime = q(end, 1:6) + alpha*tau; % apply gradient step
    q      = [q; qprime];
    
end

% interpolate joint trajectory for q6
t  = linspace(t1, t2, size(q,1)); % time steps
q6 = linspace(qs(6),qdes(6),size(q,1));
q(:,6) = q6';

% apply spline interpolation over all joint angle steps
qref = spline(t, q');

end