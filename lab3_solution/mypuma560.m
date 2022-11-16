% defines a PUMA560 robot given its DH parameters matrix (theta, d, a, alpha)
function myrobot = mypuma560(DH)

% the DH matrix should have at least one entry
assert(size(DH,1)  > 1);
myrobot = Link('revolute', DH(1, :));

% add the rest of the links to the robot
for link = 2:size(DH, 1)
    myrobot = myrobot + Link('revolute', DH(link, :));
end

end