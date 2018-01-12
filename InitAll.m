addpath('./params');
addpath('./ClusterData/');
load calib_vicon_xtion.mat

% Is it the drill model?
Drill = strcmp(Folder,'./drill');

% Load model
load(strcat(Folder,'/model.mat'));

% Load cam pose
load(strcat(Folder,'/pose.mat'));

PoseCtr = 1;
AllPts = zeros(3,0);
AllPtsRGB = zeros(0,3);
