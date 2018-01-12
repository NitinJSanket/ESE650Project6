function [BestInlierSet, BestOutlierSet, Plane,r,g,b] = RANSAC(Pts, NumIter, Thld, InlierFrac,r,g,b)
% Runs RANSAC to estimate majority plane from 3D point cloud
% ------------------ Inputs -----------------------------------------
% Pts: Raw point set, need minimum of 3 points, Pts is 3XN
% NumIter: Maximum Number of Iterations
% Thld: RANSAC Threshold to count as an inlier
% InlierFrac: Inlier Fractions
% ------------------ Outputs ----------------------------------------
% BestInlierSet: Points on the plane
% BestOutlierSet: Points not on the plane
% Code by: Nitin J. Sanket (nitinsan@seas.upenn.edu)

if(nargin<4)
    InlierFrac = 0.999;
    if(nargin<3)
        Thld = 0.1;
        if(nargin<2)
            NumIter = 100;
        end
    end
end

NumPts = length(Pts);
BestInlierSet = [];
if(nargout==2)
    BestOutlierSet = [];
end
if(nargout==3)
    BestOutlierSet = [];
    Plane = [];
end
for iter = 1:NumIter
    % Pick 3 points at random
    PickIdxs = randperm(NumPts,3);
    PtsNow = Pts(:,PickIdxs);
    % Points are A,B and C
    AB = PtsNow(:,2)-PtsNow(:,1);
    AC = PtsNow(:,3)-PtsNow(:,1);
    Normal = cross(AB,AC);
    % Plane equation is ax+by+cz+d=0;
    d = -(PtsNow(:,1)'*Normal);
    % Evaluate all points onto the estimated plane
    EstPts = abs(bsxfun(@plus,sum(bsxfun(@times,Pts,Normal),1),d))./norm(Normal);
    InlierPts = Pts(:,EstPts<=Thld);
    
    % Check to see if you got more inlier points, else retain old one
    if(length(InlierPts)>length(BestInlierSet))
        BestInlierSet = InlierPts;
        if(nargout>=2)
            OutlierPts = Pts(:,abs(EstPts)>Thld);
            BestIdxs = abs(EstPts)>Thld;
            BestOutlierSet = OutlierPts;
            Plane = [Normal;d];
        end
    end
    
    % If you have a huge number of inliers terminate!
    if(length(InlierPts)/NumPts>=InlierFrac)
        disp(['Terminating Early at iteration ', num2str(iter), ', ' , 'Inlier Ratio ' ,num2str(length(InlierPts)/NumPts)]);
        r = r(BestIdxs);
        g = g(BestIdxs);
        b = b(BestIdxs);
        return;
    end
    if(iter==NumIter)
        r = r(BestIdxs);
        g = g(BestIdxs);
        b = b(BestIdxs);
        disp(['Reached Maximum of ', num2str(NumIter), ' iterations, Terminating....']);
    end
end
end
