function linelaser_calibration()
%linelaser_calibration ラインレーザのCalibration
%   
    %動画からFrameを保存する   
    load fishparams.mat fisheyeParams
    load setup.mat linelaser_dir laser_step squareSize laser_time margin bright_thr
    vidObj = VideoReader(linelaser_dir);
    allFrame = read(vidObj);
    
    %チェッカーボード検出
    laserimg = allFrame(:,:,:,int16(laser_time(1)*vidObj.FrameRate):laser_step:int16(laser_time(2)*vidObj.FrameRate));
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
       
       %レーザ点群取得
       rect =int16([min(imagePoints(:,:,i))-margin, max(imagePoints(:,:,i))-min(imagePoints(:,:,i))+2*margin]); % チェッカーボード周辺の矩形
       Jtrim = imcrop(J, rect); % Trimming
       [Yd, Xd] = find(Jtrim > bright_thr);  
       BrightPoints = [Xd, Yd] + double([rect(1), rect(2)]);% 閾値以上の輝点の画像座標
       
       %レーザの輝点の画像座標->World座標系に変換
       LaserworldPoints = pointsToWorld(fisheyeParams.Intrinsics, RotMatrix(:,:,i),TransVec(i,:),BrightPoints);
       newworldPoints = [LaserworldPoints, zeros(size(LaserworldPoints,1),1)];
       %World->Camera座標系に変換
       cameraPoints = newworldPoints * RotMatrix(:,:,i) + TransVec(i,:);
       All_cameraPoints = [All_cameraPoints;cameraPoints];
       
       %中央のレーザ映る場所の輝点抽出
    end
    
    %レーザ輝点群を最小二乗法で一つの平面を出力
    func = @(param)calcplane_func(param, All_cameraPoints);
    flg = 1;
    false_cnt = 1;
    while flg
        x0 = -rand(1,3)*0.01*false_cnt;
        options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
        [Sol,fval,exitflag,output] = fminsearch(func,x0,options);
        if exitflag==1&&fval<3000
            flg = 0;
            false_cnt = 1;
        else
            false_cnt = false_cnt +1;
        end
        if false_cnt >10
            break;
        end  
    end
    
    
    
    
    disp(fisheyeParams.Intrinsics);
end