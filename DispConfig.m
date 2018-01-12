if(P2P)
    disp('Running using Point to Point, Slower and Less accurate approach....');
else
    disp('Running using Point to Plane, Faster and More accurate approach....');
end

if(Subsample)
    disp('Subsampling is enabled....');
else
    disp('Subsampling is disabled....');
end

if(Truncation)
    disp('Truncation is enabled....');
else
    disp('Truncation is disabled....');
end


if(PlotFlag)
    disp('Plotting Results....');
    if(VideoFlag)
        disp('Recording Video....');
    else
        disp('Not Recording Video....');
    end
    disp('Not Plotting Results....');
end

disp(['Evaluating every ', num2str(Step),' frames....']);