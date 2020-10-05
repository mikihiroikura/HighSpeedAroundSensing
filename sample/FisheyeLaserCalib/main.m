%%%　全体の動作を決定する関数

% 初期化
make_setup();
disp('Setup Finished.');

% 魚眼カメラのキャリブレーション
fisheye_calibration();
disp('Camera Calibration finished.');

% 魚眼カメラ画像の円のハフ変換
% fisheye_houghcircle();
% disp('Fisheye circle detection finished.');

% ラインレーザのCalibration
linelaser_calibration();
disp('Linelaser Calibration finished.');

% 平面パラメータと参照面輝点画像座標の関連付け
laserparams_imterpolate();
disp('LaserPlane parameters ImterpAllolated.');