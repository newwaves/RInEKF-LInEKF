function [XbkTF, Fk, Gk] = InEKF_PropagationLeft( nFrm, XkTF, ParaS )

if ParaS.isFirst == 1  %nFrm == 1
    XbkTF = XkTF; % [R, t; 0 0 0 1]
    Fk = eye(6);
    Gk = zeros(6,6);
else
    if size(ParaS.vOdoTF, 3) > 500 && ParaS.dFrm >= 2
        PreTF = ParaS.vOdoTF(:,:,nFrm - ParaS.dFrm) * ParaS.CalTF; % convert to LiDAR system
        CurTF = ParaS.vOdoTF(:,:,nFrm)              * ParaS.CalTF;
    else
        % B = load('KITTI00/OdoPosedF5.mat');
        PreTF = ParaS.vOdoTF(:,:,nFrm - 1) * ParaS.CalTF; % convert to LiDAR system
        CurTF = ParaS.vOdoTF(:,:,nFrm)     * ParaS.CalTF;
    end

    Uk = PreTF \ CurTF; % inv(PreTF) * CurTF;
    XbkTF = XkTF * Uk;  % X * U
    % Left Invariant EKF
    Fk = Adjoint(inv(Uk));
    Gk = eye(6);
%     % Right Invariant EKF
%     Fk = eye(6);
%     Gk = Adjoint(XbkTF);
end
tmpPose = CTF2Pose(XbkTF);
TmpQ = eul2quat(tmpPose(4:6)');
TmpR = quat2rotm(TmpQ);
TmpR = eul2rotm(rotm2eul(TmpR));
TmpT = XbkTF(1:3,end);
XbkTF = [TmpR,TmpT;0 0 0 1];
end
