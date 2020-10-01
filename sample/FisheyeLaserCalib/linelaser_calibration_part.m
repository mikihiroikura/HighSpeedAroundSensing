function linelaser_calibration_part(linelaser_file_no,Opt)
%linelaser_calibration_part 自分でCalibrationする平面を決められる
%   
    %.matファイルから変数群の呼び出し  
    load fishparams.mat fisheyeParams
    load setup.mat linelaser_folder laser_step squareSize laser_time_margin ...
    margin bright_thr ref_circle_radi ref_thr ref_arcwidth linelaser_file_num
    load linelaserparams.mat All_planeparams All_refPoints ref_center ref_radi All_cameraPoints All_fvals All_PointsCnts

    %画像群呼び出し
    k = linelaser_file_no;
    linelaser_dir = strcat(strcat(linelaser_folder, int2str(k)),'.mp4');
    vidObj = VideoReader(linelaser_dir);
    allFrame = read(vidObj);
    laserimg = allFrame(:,:,:,int16(laser_time_margin*vidObj.FrameRate):laser_step:int16((vidObj.Duration-laser_time_margin)*vidObj.FrameRate));

    %チェッカーボード検出
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
    OnePlane_cameraPoints = [];    
    detectedimgs = laserimg(:,:,:,imagesUsed);
    for i=1:size(detectedimgs, 4)
       J = detectedimgs(:,:,:,i);
       BW = imbinarize(J, 0.8);
       BW = uint8(BW);
       J = J.*BW;

       %レーザ点群取得
       rect =int16([min(imagePoints(:,:,i))-margin, max(imagePoints(:,:,i))-min(imagePoints(:,:,i))+2*margin]); % チェッカーボード周辺の矩形
       Jtrim = imcrop(J, rect); % Trimming
       [Yd, Xd] = find(Jtrim(:,:,1) > bright_thr);  
       BrightPoints = [Xd, Yd] + double([rect(1), rect(2)]);% 閾値以上の輝点の画像座標

       %レーザの輝点の画像座標->World座標系に変換
       LaserworldPoints = pointsToWorld(fisheyeParams.Intrinsics, RotMatrix(:,:,i),TransVec(i,:),BrightPoints);
       newworldPoints = [LaserworldPoints, zeros(size(LaserworldPoints,1),1)];
       %World->Camera座標系に変換
       cameraPoints = newworldPoints * RotMatrix(:,:,i) + TransVec(i,:);
       OnePlane_cameraPoints = [OnePlane_cameraPoints;cameraPoints];

       %中央のレーザ映る場所の輝点抽出
    end

    %レーザ輝点群を最小二乗法で一つの平面を出力
    if k<=7 || k>=21
        norm = 0;
        maxcnt = 3;
    else
        norm = 0.001;
        maxcnt = 10;
    end
    func = @(param)calcplane_func(param, OnePlane_cameraPoints);
    min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norm;%0.001
        options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
        [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
        if exitflag==1&&fval<3000
            opt_before_planeparams = planeparams;
            break;
        elseif exitflag==1
            if min_fval > fval
                min_fval = fval;
                opt_before_planeparams = planeparams;
            end
        end
    end

    if Opt
        %外れている輝点を取り除く 閾値10
        dist = abs(1-(opt_before_planeparams(1).*OnePlane_cameraPoints(:,1)+opt_before_planeparams(2) ...
                .*OnePlane_cameraPoints(:,2)+opt_before_planeparams(3).*OnePlane_cameraPoints(:,3))) ...
                ./(opt_before_planeparams(1)^2+opt_before_planeparams(2)^2+opt_before_planeparams(3)^2)^0.5;
        Opt_OnePlane_cameraPoints = OnePlane_cameraPoints(dist<10,:);

        %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
        func = @(param)calcplane_func(param, Opt_OnePlane_cameraPoints);
        opt_min_fval = 1e+20;
        for cnt = 1:maxcnt
            x0 = -rand(1,3)*norm;%0.001
            options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
            [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
            if exitflag==1&&fval<3000
                opt_after_planeparams = planeparams;
                break;
            elseif exitflag==1
                if opt_min_fval > fval
                    opt_min_fval = fval;
                    opt_after_planeparams = planeparams;
                end
            end
        end
    end
    if Opt
        opt_planeparams = opt_after_planeparams;
    else
        opt_planeparams = opt_before_planeparams;
    end
    
    %結果の保存
    All_planeparams(k,:) = opt_planeparams;
    save linelaserparams.mat All_planeparams All_refPoints ref_center ref_radi All_cameraPoints All_fvals All_PointsCnts

end

% デバッグ：Calibrationした平面の結果を図で確認
% All_PointsID = [0;size(OnePlane_cameraPoints,1)];
% f = figure;
% for i = 1
%     X = OnePlane_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),1);
%     Y = OnePlane_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),2);
%     Z = OnePlane_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),3);
%     scatter3(X,Y,Z);
%     hold on
%     graphX = linspace(min(X),max(X),500);
%     graphY = linspace(min(Y),max(Y),500);
%     [gX,gY] = meshgrid(graphX,graphY);
%     gZ = -opt_planeparams(i,1)/opt_planeparams(i,3)*gX-opt_planeparams(i,2)/opt_planeparams(i,3)*gY+1/opt_planeparams(i,3);
%     mesh(gX,gY,gZ);
% end

% 外れ値処置以前の平面検出結果
% min_fval/size(OnePlane_cameraPoints,1);

% 外れ値処理以後の平面検出結果
% opt_min_fval/size(Opt_OnePlane_cameraPoints,1);

