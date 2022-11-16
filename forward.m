% returns the final homogeneous transformation matrix over the sequence of transformations
% parametrized by the jointvals row-vector
function H = forward(jointvals, myrobot)

H = myrobot.A(1:size(jointvals, 1), jointvals);
H = H.T;

end