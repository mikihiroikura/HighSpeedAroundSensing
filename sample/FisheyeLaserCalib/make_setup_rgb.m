function make_setup_rgb()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    %fisheye_calibration()
    video_folder = './videos/fisheye/';
    video_name = '202103092152_video_qcav.mp4';
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
    linelaser_file_num = 1:35;
    laser_step = 2;
    laser_time_margin = 4; % レーザがチェッカーボードに映っている時間
    margin = 30; % チェッカーボード周辺を抽出する際のマージン
    bright_r_thr = 200;%レーザ輝度の閾値
    ref_circle_radi = [50 80];%参照面の円の検出する半径の範囲
    ref_r_thr = 100;%参照面の輝点閾値
    ref_arcwidth = 6;%参照面の円の検出する円弧の幅
    ref_arcwidth_margin = 2;
    
    %ラインレーザの補間用変数
    %laserparams_interpolate()
    interpolate_outputfile = strcat('./calib_result/linelaser/',strcat(datestr(now,form),'_laserinterpolparam.csv'));
    
    %参照面の式を導出する
   refplane_dir = 'videos/refplane/202103161642_video.mp4';
   ref_step = 10;
   %ref_squareSize = 32;
   ref_squareSize = 28.3;
   %ref_diffs = [-482,-42.5,-302];
   ref_diffs = [0,0,5];
   ref_norm = 0;
   ref_maxcnt = 3;
    
    
    save setup_rgb.mat video_dir fish_step squareSize fishparamfile ...
linelaser_folder laser_step laser_time_margin margin bright_r_thr ref_circle_radi ref_r_thr ref_arcwidth...
linelaser_file_num interpolate_outputfile ref_arcwidth_margin ...
refplane_dir ref_squareSize ref_diffs ref_norm ref_maxcnt ref_step
end

