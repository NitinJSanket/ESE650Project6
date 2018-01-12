RCorr = HVicToObj(1:3,1:3);
for i = 1:size(Pose,3)
    Eul(i,:) = rotm2eul(Pose(1:3,1:3,i));
end

i = 1;
for k = 1:Step:length(dir([Folder,'/rgb/*.png']))
    EulVic(i,:) = quat2eul(pose(k,3:6)); % camera model orientation w.r.t. reference coordinate frame
    i = i+1;
end

if(Drill)
    figure,
    plot(Eul(:,1));
    hold on;
    plot(EulVic(:,1));
    plot(Eul(:,2));
    plot(bsxfun(@plus,-EulVic(:,2),2.*mean(EulVic(:,2))));
    plot(Eul(:,3));
    plot(bsxfun(@plus,-EulVic(:,3),2.*mean(EulVic(:,3))));
    legend('Yaw Est','Yaw Vic','Pitch Est','Pitch Vic','Roll Est','Roll Vic');
    xlabel('Frame Number');
    ylabel('Angles (rad)');
else
    figure,
    plot(Eul(:,1));
    hold on;
    plot(EulVic(:,1));
    plot(Eul(:,2));
    plot(EulVic(:,2));
    plot(Eul(:,3));
    plot(EulVic(:,3));
    legend('Yaw Est','Yaw Vic','Pitch Est','Pitch Vic','Roll Est','Roll Vic');
    xlabel('Frame Number');
    ylabel('Angles (rad)');
end