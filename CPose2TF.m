function TF = CPose2TF(Pose)
% Pose - 6 x 1
if size(Pose, 1) == 1
    t = Pose(1:3)';
    R = eul2rotm(Pose(4:6));% ct = [cz cy cx] and st = [sy sy sx]
else % size(Pose, 1) == 6
    t = Pose(1:3);
    R = eul2rotm(Pose(4:6)');% ct = [cz cy cx] and st = [sy sy sx]
end

TF = [R,t; 0 0 0 1];
end
