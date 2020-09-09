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
    All_cameraPoints = [];    
    detectedimgs = laserimg(:,:,:,imagesUsed);
    for i=1:size(detectedimgs, 4)
       J = detectedimgs(:,:,:,i);
       BW = imbinarize(J, 0.8);
       BW = uint8(BW);
       J = J.*BW;
       %ここでどのように輝度重心を取得するか
       BrightPoints = [0,0];
       
       %レーザの輝点の画像座標->World座標系に変換
       worldPoints = pointsToWorld(fisheyeParams.Intrinsics, RotMatrix(:,:,i),TransVec(i,:),BrightPoints);
       newworldPoints = [worldPoints, zeros(size(worldPoints,1),1)];
       %World->Camera座標系に変換
       cameraPoints = newworldPoints * RotMatrix(:,:,i);
       All_cameraPoints = [All_cameraPoints;cameraPoints];
       
       %中央のレーザ映る場所の輝点抽出
    end
    
    %レーザ輝点群を最小二乗法で一つの平面を出力
    
    
    
    
    disp(fisheyeParams.Intrinsics);
end