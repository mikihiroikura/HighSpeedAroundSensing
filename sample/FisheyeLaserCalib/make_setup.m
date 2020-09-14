function make_setup()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    %fisheye_calibration()
    video_folder = './videos/fisheye/';
    video_name = '202009012005_video.mp4';
    video_dir = strcat(video_folder, video_name);
    fish_step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    
    %LinelaseのCalibration用変数
    %linelaser_calibration()
    linelaser_folder = './videos/linelaser/';
    linelaser_name = '202009102032_video.mp4';
    linelaser_dir = strcat(linelaser_folder, linelaser_name);
    laser_step = 2;
    laser_time = [3,18]; % レーザがチェッカーボードに映っている時間
    margin = 30; % チェッカーボード周辺を抽出する際のマージン
    bright_thr = 200;%レーザ輝度の閾値
    
    save setup.mat video_dir fish_step squareSize fishparamfile linelaser_dir laser_step laser_time margin bright_thr
end

