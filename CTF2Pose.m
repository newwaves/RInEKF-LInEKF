function Pose = CTF2Pose(TF)
% Pose - 6 x 1
Pose = [TF(1:3,end)', rotm2eul(TF(1:3,1:3))]';
end
