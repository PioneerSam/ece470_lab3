function myrobot = mypuma560(DH)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

joint_num = size(DH, 1);

myrobot = SerialLink(DH, 'name', 'This is the AWESOME PUMA560 Robot');

% for i = 1:joint_num
%     myrobot = myrobot + Link('revolute', DH(i, :));
% end 

end
