function [vTraPts, vHdlPts, vMapPts, vMapNor] = FindCorrespondencesV2(kdMap, ptMap, ptHdl, RegTF, InlierRatio, Xbk)

vtranPts = myRigidTransform(ptHdl.Location, RegTF(1:3,1:3), RegTF(1:3,end));
%%
upperBoundRatio = max(1, round(InlierRatio(1) * ptHdl.Count)); 
%Find the correspondence
[indices, dists] = knnsearch(kdMap, vtranPts, 'dist','euclidean');
% Remove outliers
keepInlierA = false(ptHdl.Count, 1);
[~, idx] = sort(dists);
%%
vhdlPts1 = myRigidTransform(ptHdl.Location, Xbk(1:3,1:3), Xbk(1:3,end));
vmapPts1 = ptMap.Location(indices,:);
% vmapNor1 = ptMap.Normal(indices,:);
dhdlmap = vhdlPts1 - vmapPts1;
squaredDist = sum(dhdlmap.^2, 2);
DistThr = sqrt(median(squaredDist));%sqrt(median(squaredDist))
%%
upperBoundDist = length(find(dists < DistThr)); 

upperBound = min(upperBoundRatio, upperBoundDist); % added 2020-11-03

keepInlierA(idx(1:upperBound)) = true;
inlierIndicesB = indices(keepInlierA);
vTraPts = vtranPts(keepInlierA,:); % Here is different from CorrectStepModel
vHdlPts = ptHdl.Location(keepInlierA,:);
vMapPts = ptMap.Location(inlierIndicesB,:);
vMapNor = ptMap.Normal(inlierIndicesB,:);

end