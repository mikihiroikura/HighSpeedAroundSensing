function make_setup_rgb()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    %fisheye_calibration()
    video_folder = './videos/fisheye/';
    video_name = '202011251933_video.mp4';
    video_dir = strcat(video_folder, video_name);
    fish_step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/fisheye/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    
    %魚眼カメラの円検出用変数
%     fishbright_name = '202009012005_video.mp4';
%     fishbright_dir = strcat(video_folder, fishbright_name);
%     fish_circle_radi = [300 550];
    
    %LinelaseのCalibration用変数
    %linelaser_calibration()
    linelaser_folder = './videos/linelaser/';
    linelaser_file_num = 1:23;
    laser_step = 2;
    laser_time_margin = 4; % レーザがチェッカーボードに映っている時間
    margin = 30; % チェッカーボード周辺を抽出する際のマージン
    bright_r_thr = 150;%レーザ輝度の閾値
    ref_circle_radi = [25 45];%参照面の円の検出する半径の範囲
    ref_r_thr = 220;%参照面の輝点閾値
    ref_arcwidth = 10;%参照面の円の検出する円弧の幅
    
    %ラインレーザの補間用変数
    %laserparams_interpolate()
    interpolate_outputfile = strcat('./calib_result/linelaser/',strcat(datestr(now,form),'_laserinterpolparam.csv'));
    
    save setup_rgb.mat video_dir fish_step squareSize fishparamfile ...
linelaser_folder laser_step laser_time_margin margin bright_r_thr ref_circle_radi ref_r_thr ref_arcwidth...
linelaser_file_num interpolate_outputfile %fishbright_dir fish_circle_radi
end

