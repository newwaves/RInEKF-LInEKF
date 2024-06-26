function [Xk, Ck] = InEKF_UpdateRight(kdMap, ptMap, ptHdl, Cbk, XbkTF, ParaS)

coresTF = ParaS.CorresTF;
if strcmp(ParaS.DataAssociM, 'NDT') % in LongTerm02, NDT is not limited by distance theshold
    [~, vZk, vMapPts, ~] = FindCorrespondences(kdMap, ptMap, ptHdl, coresTF, 1, ParaS.DistThr);
else % added 2023.08.11
    [~, vZk, vMapPts, ~] = FindCorrespondencesV2(kdMap, ptMap, ptHdl, coresTF, 1, XbkTF);
end
NumRatio = min(1, ParaS.ptNum ./ size(vZk, 1));
[ptTmp,vIdx] = pcDownSample(pointCloud(vZk), 'random', NumRatio); % about 3000 points
vZk = ptTmp.Location;  % keep 2000 points
vMapPts = vMapPts(vIdx, :);
%
% added 2020-04-30
if size(vMapPts, 1) > 20   % the prediction value of observed points
    % right - invariant
    vZbk = [vZk, ones(size(vZk, 1), 1)];
    vtmpPts = XbkTF * vZbk';
    vZbk = vtmpPts(1:3, :)';    
    %%
    Qk = ParaS.ScaleC .* diag(ParaS.Cov);  % 2020-05-07
    eState = eye(4); %ParaS.errorS; %
    for i = 1 : 1 : size(vZbk,1)
        Hx = [skew(vMapPts(i,:)), -eye(3)]; % right2        
        Sk = Hx * Cbk * Hx' + Qk;
        Kk = Cbk * Hx' / Sk;
        eState = Exp(Kk * (vZbk(i,:)' - vMapPts(i,:)')) * eState; % right2
        Cbk = (eye(6) - Kk * Hx) * Cbk;
    end
    %-----------------------------------------------------
    Xk = eState * XbkTF; % right-invariant
    Ck = Cbk;
    %-----------------------------------------------------
else
    Xk = XbkTF;
    Ck = Cbk;
end
end
%%
function state =  Exp(epslion) % R, t
R = expm(skew(epslion(1:3)));
t = epslion(4:6);
state = [R, t; 0 0 0 1];
end