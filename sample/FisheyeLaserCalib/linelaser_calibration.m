function linelaser_calibration()
%linelaser_calibration ラインレーザのCalibration
%   
    %動画からFrameを保存する   
    load fishparams.mat fisheyeParams
    load setup.mat linelaser_dir laser_step squareSize
    vidObj = VideoReader(linelaser_dir);
    allFrame = read(vidObj);
    
    %チェッカーボード検出
    laserimg = allFrame(:,:,:,1:laser_step:size(allFrame,4));
    [imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(laserimg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    %チェッカーボード(World)toカメラの外部パラメータの計算
    RotMatrix = zeros(3,3,size(imagePoints,3));
    TransVec = zeros(size(imagePoints,3), 3);
    for i=1:size(imagePoints,3)
        [R,t] = extrinsics(imagePoints(:,:,i),worldPoints,fisheyeParams.Intrinsics);
        RotMatrix(:,:,i) = R;
        TransVec(i,:) = t;
    end
    
    %レーザの輝点の画像座標を計算
    
    
    %レーザの輝点の画像座標->カメラ座標系に変換
    
    
    disp(fisheyeParams.Intrinsics);
end