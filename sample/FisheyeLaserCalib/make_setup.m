function make_setup()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    video_folder = './videos/fisheye/';
    video_name = '202009012005_video.mp4';
    video_dir = strcat(video_folder, video_name);
    step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    
    %LinelaseのCalibration用変数
    linelaser_folder = './videos/linelaser/';
    linelaser_name = '202009041825_video.mp4';
    linelaser_dir = strcat(linelaser_folder, linelaser_name);
    
    save setup.mat video_dir step squareSize fishparamfile linelaser_dir
end

