function linelaser_calibration()
%linelaser_calibration ラインレーザのCalibration
%   
    %動画からFrameを保存する   
    load fishparams.mat fisheyeParams
    
    disp(fisheyeParams.Intrinsics);
end