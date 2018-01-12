for k = 1:Step:length(dir([Folder,'/rgb/*.png']))
    frame = pose(k,1);
    t = pose(k,2);
    qcam = pose(k,3:6); % camera model orientation w.r.t. reference coordinate frame
    xcam = pose(k,7:9); % camera model position w.r.t. reference coordinate frame
    if(k==1)
        HCinWInit = [quat2rotm(qcam),xcam'.*10;0,0,0,1];
    end
    
    % % Load images
    depth = imread(strcat(Folder,'/depth/',sprintf('%06d.png',k)));
    depth(depth>1000|depth<100) = 0;
    rgb = imread(strcat(Folder,'/rgb/',sprintf('%06d.png',k)));
    %
    % % point cloud
    [pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(depth, rgb, './params/calib_xtion.mat');
    Pts = transpose(s_cam3*[pcx pcy pcz]*[0 0 1;-1 0 0;0 -1 0]'*R_cv3 + repmat(t_cv3,length(pcx),1));
    
    %% Run RANSAC
    NumIter = 1000;
    Thld = 15;
    InlierFrac = 0.96;
    [InlierPts, OutlierPts, Plane, r, g, b] = RANSAC(Pts, NumIter, Thld, InlierFrac,r,g,b);
    
    % Remove outlier points (if any)
    MeanPt = mean(OutlierPts,2);
    Diff = sum(bsxfun(@minus,OutlierPts,MeanPt).^2);
    MeanPtModel = mean(Mdata,2);
    DiffModel = sum(bsxfun(@minus,Mdata,MeanPtModel).^2);
    Thld = max(DiffModel).*1.2;
    OutlierPts = OutlierPts(:,Diff<=Thld);
    r = r(Diff<=Thld);
    g = g(Diff<=Thld);
    b = b(Diff<=Thld);
    
    % Calculate Pose of the camera from RANSAC plane assuming plane was horizontal
    % First, calculate the orientation with repsect to ideal plane
    % direction
    %     RTot = eul2rotm([0,pi,0])*vrrotvec2mat(vrrotvec(Plane(1:3)',[0,0,1]));
    % [0 0 1;-1 0 0;0 -1 0]'*RTot' for Cam Pose
    
    %% ICP
    OrgData = Mdata;
    Z = Mdata;
    NIter = 25;
    OutlierPts = bsxfun(@minus,OutlierPts,mean(OutlierPts,2));
    if(k==1)
        
        if(Drill)
            % For Drill
            RTot = eye(3);
            TTot = [0,0,0]';
            load('KMeans512ClustersDrill.mat');
        else
            % For Liquid Container
            RTot = eul2rotm([0,pi/4,0]);
            TTot = [0,-50,50]';
            load('KMeans512ClustersContainer.mat');
        end
        
        HTot = [RTot,TTot;0,0,0,1];
        T = TTot;
    end
    PrevErr = Inf;
    
    NumClusters = 512;
    ptCloud = pointCloud(OrgData');
    Normals = transpose(pcnormals(ptCloud));
    %     [KMeansIdx,KMeansC] = kmeans(Normals',NumClusters,'Distance','cosine','MaxIter',1000,'Display','iter');
    if(Subsample)
        KMeansIdx = [];
        for cluster = 1:NumClusters
            KMeansIdx(:,cluster) = pdist2(Normals',KMeansC(cluster,:),'cosine');
        end
        disp('Cluster Assignment Complete....');
        
        [~,KMeansIdx] = min(KMeansIdx,[],2);
        SelIdxs = [];
        
        % Clustering And Sub-sampling
        SamplePerClass = 10;
        for cluster = 1:NumClusters
            CurrIdx = (KMeansIdx==cluster); % All points in current cluster
            % You have very few samples in the current cluster, return all the
            % samples
            if(sum(CurrIdx)<=SamplePerClass)
                SelIdxs = [SelIdxs;find(CurrIdx)];
                continue;
            end
            RandSamples = randsample(sum(CurrIdx),SamplePerClass);
            NonZeroIdxs = find(CurrIdx);
            SelIdxs = [SelIdxs;NonZeroIdxs(RandSamples)];
        end
        Normals = Normals(:,SelIdxs);
        Mdata = Mdata(:,SelIdxs);
        OrgData = OrgData(:,SelIdxs);
    end
    
    
    tic
    for iter = 1:NIter
        if(iter==1)
            RotPts = bsxfun(@plus,RTot*OutlierPts,TTot);
        else
            RotPts = bsxfun(@minus,RotPts,T);
            RotPts = R'*RotPts;
            Err = sqrt(sum(sum((Mdata(:,CorresIdx)-RotPts(:,KeepIdxs)).^2)));
            clc;
            fprintf('Error in iteration %d was %f',iter-1,Err);
            if(abs(Err-PrevErr)<=1e-3)
                fprintf('Change in error was %f, breaking....',abs(Err-PrevErr));
                break;
            end
            PrevErr = Err;
            HTot = [R',-R'*T;0,0,0,1]*HTot;
            RTot = R'*RTot;
            TTot = R'*(TTot-T);
        end
        % Association of Points
        CorresIdx = knnsearch(Mdata',RotPts'); % Finds closest point in Mdata for points in RotPts
        Dist = sqrt(sum((Mdata(:,CorresIdx)-RotPts).^2));
        MaxDist = max(Dist);
        
        if(Truncation)
            % Keep only top 85% points based on distance
            KeepIdxs = Dist<=0.85.*MaxDist;
            CorresIdx = CorresIdx(KeepIdxs);
        else
            KeepIdxs = 1:length(Dist);
        end
        if(P2P)
            [R, T] = ICP(Mdata(:,CorresIdx), RotPts(:,KeepIdxs));
        else
            [R, T] = ICP(RotPts(:,KeepIdxs), Mdata(:,CorresIdx), 'PointToPlane', Normals(:,CorresIdx));
        end
        if(PlotFlag)
            subplot 121
            if(k>1)
                pcshow(AllPts',AllPtsRGB);
            end
            xlabel('x'); ylabel('y'); zlabel('z');
            axis equal;
            subplot 122
            pcshow(Z','r');
            hold on;
            pcshow(RotPts','b');
            axis equal;
            suptitle(num2str(k));
            drawnow;
        end
        if(VideoFlag)
            axes('Position',[0 0 1 1],'Visible','off');
            writeVideo(Vid, getframe(gcf));
        end
    end
    toc
    
    HObjInCam = HTot;
    HObjInW = HCinWInit/HObjInCam;
    ObjInW = bsxfun(@plus,HObjInW(1:3,1:3)*RotPts,HObjInW(1:3,4));
    HCamInW = HObjInCam*HObjInW;
    
    if(PoseCtr==1)
        HVicToObj = HCinWInit\HTot;
        RCorr = HVicToObj(1:3,1:3);
        TCorr = HVicToObj(1:3,4);
    end
    Pose(:,:,PoseCtr) = HTot*[RCorr',[0,0,0]';0,0,0,1];
    PoseCtr = PoseCtr+1;
    
    AllPts = [AllPts,RotPts];
    AllPtsRGB = [AllPtsRGB;[r,g,b]./255];
end

if(VideoFlag)
    close(Vid);
end