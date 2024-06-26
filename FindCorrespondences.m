function [vTraPts, vHdlPts, vMapPts, vMapNor] = FindCorrespondences(kdMap, ptMap, ptHdl, InitTF, InlierRatio, DistThr)

vtranPts = myRigidTransform(ptHdl.Location, InitTF(1:3,1:3), InitTF(1:3,end));
%%
upperBoundRatio = max(1, round(InlierRatio(1) * ptHdl.Count)); 
%Find the correspondence
[indices, dists] = knnsearch(kdMap, vtranPts, 'dist','euclidean');
% Remove outliers
keepInlierA = false(ptHdl.Count, 1);
[~, idx] = sort(dists);
upperBoundDist = length(find(dists < DistThr)); 

upperBound = min(upperBoundRatio, upperBoundDist); % added 2020-11-03

keepInlierA(idx(1:upperBound)) = true;
inlierIndicesB = indices(keepInlierA);
vTraPts = vtranPts(keepInlierA,:); % Here is different from CorrectStepModel
vHdlPts = ptHdl.Location(keepInlierA,:);
vMapPts = ptMap.Location(inlierIndicesB,:);
vMapNor = ptMap.Normal(inlierIndicesB,:);

end