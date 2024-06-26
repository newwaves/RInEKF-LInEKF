function vTFPose = CvEul2vTF(Pose)
Len = size(Pose,1);
vTFPose = zeros(4,4,Len);
for i = 1 : 1 : Len
    tmpT = Pose(i, 1:3);
    tmpR = eul2rotm(Pose(i, 4:6), 'ZYX');
    vTFPose(:,:,i) = [tmpR,tmpT'; 0 0 0 1];
end
end