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
    
    save setup.mat video_dir step squareSize fishparamfile
end

