function make_setup()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    video_folder = './videos/fisheye/';
    video_name = '202009012005_video.mp4';
    video_dir = strcat(video_folder, video_name);
    fish_step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    
    %LinelaseのCalibration用変数
    linelaser_folder = './videos/linelaser/';
    linelaser_name = '202009041828_video.mp4';
    linelaser_dir = strcat(linelaser_folder, linelaser_name);
    laser_step = 10;
    
    save setup.mat video_dir fish_step squareSize fishparamfile linelaser_dir laser_step
end

