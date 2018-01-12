%% Wrapper for 3D Object Scanning/Pose Estimation using RGBD Data and Fast-ICP
% Code by: Nitin J. Sanket (nitinsan@seas.upenn.edu)
% NOTE: CODE CLEARS ALL VARIABLES IN THE WORKSPACE!
% Activate the required flags

clc
clear all
close all
warning off;

%% Set Data Path and Load required data
Folder = './liq_container';
InitAll;

%% Setup the Flags Needed
Subsample = 1; % Do you want to Subsample in Normal Space?, Default: Yes (VERY FAST!)
Truncation = 1; % Do you want to truncate at every setp?, Default: Yes (VERY FAST AND ROBUST TO OUTLIERS!)
P2P = 0; % Do you want to use Point 2 Point (SLOWER AND LESS ACCURATE) or Point 2 Plane, Default: Point 2 Plane (VERY FAST!)
Step = 10; % Frame Skip factor
PlotFlag = 0; % Display Results?, Default: Yes
VideoFlag = 0; % Record Video?, Default: Yes, Active only when Plot Flag is active
if(PlotFlag==0)
    VideoFlag = 0;
end
if(VideoFlag)
    Vid = VideoWriter(['Video',Folder(3:end),num2str(P2P),'.avi']);
    open(Vid);
end
DispConfig;

%% Run ICP and aggregate points
RunObjectScanner;

%% Denoise and Display
figure,
Denoised = pcdenoise(pointCloud(AllPts'),'Threshold',2,'NumNeighbors',10);
CorresIdx = knnsearch(AllPts',Denoised.Location);
subplot 121
pcshow(Denoised.Location,AllPtsRGB(CorresIdx,:));
title('Denoised');
subplot 122
pcshow(AllPts',AllPtsRGB);
title('Org');
saveas(gcf, ['Denoised',Folder(3:end),num2str(P2P),'.png']);

%% Display Comparisons
figure,
DispComparo;
saveas(gcf, ['Comparo',Folder(3:end),num2str(P2P),'.png']);