%%%　全体の動作を決定する関数

% 初期化
make_setup();
disp('Setup Finished.');

% 魚眼カメラのキャリブレーション
fisheye_calibration();
disp('Camera Calibration finished.');

% ラインレーザのCalibration
linelaser_calibration();
disp('Linelaser Calibration finished.');