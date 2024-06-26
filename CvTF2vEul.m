function vPose = CvTF2vEul(vTF)
Len = size(vTF,3);
vPose = zeros(Len,6);
for i = 1 : 1 : Len
    tmpTF = vTF(:,:,i);
    vPose(i,:) = [tmpTF(1:3,end)', rotm2eul(tmpTF(1:3,1:3))];
end

end