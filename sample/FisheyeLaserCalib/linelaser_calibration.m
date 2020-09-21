function linelaser_calibration()
%linelaser_calibration ラインレーザのCalibration
%   
    %動画からFrameを保存する   
    load fishparams.mat fisheyeParams
    load setup.mat linelaser_folder laser_step squareSize laser_time ...
    margin bright_thr ref_rect ref_thr linelaser_file_cnt

    %保存用の行列
    All_planeparams = [];
    All_refPoints = [];
    All_cameraPoints = [];
    All_fvals = [];
    PointsCnts = [];
    
    
    %レーザ動画の個数だけ行う
    for i=1:linelaser_file_cnt
        linelaser_dir = strcat(strcat(linelaser_folder, int2str(i)),'.mp4');
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
        func = @(param)calcplane_func(param, OnePlane_cameraPoints);
        min_fval = 1e+20;
        for cnt = 1:10
            x0 = -rand(1,3)*0.01*cnt;
            options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
            [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
            if exitflag==1&&fval<3000
                opt_planeparams = planeparams;
                break;
            elseif exitflag==1
                if min_fval > fval
                    min_fval = fval;
                    opt_planeparams = planeparams;
                end
            end
        end
        
        %外れている輝点を取り除く 閾値10
        dist = abs(1-(opt_planeparams(1).*OnePlane_cameraPoints(:,1)+opt_planeparams(2) ...
                .*OnePlane_cameraPoints(:,2)+opt_planeparams(3).*OnePlane_cameraPoints(:,3))) ...
                ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
        Opt_OnePlane_cameraPoints = OnePlane_cameraPoints(dist<10,:);
        
        %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
        func = @(param)calcplane_func(param, Opt_OnePlane_cameraPoints);
        min_fval = 1e+20;
        for cnt = 1:10
            x0 = -rand(1,3)*0.01*cnt;
            options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
            [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
            if exitflag==1&&fval<3000
                opt_planeparams = planeparams;
                break;
            elseif exitflag==1
                if min_fval > fval
                    min_fval = fval;
                    opt_planeparams = planeparams;
                end
            end
        end

        %参照面の輝点出力
        OnePlane_refPoints = [];
        for i=1:size(detectedimgs, 4)
           J = detectedimgs(:,:,:,i);
           BW = imbinarize(J, 0.8);
           BW = uint8(BW);
           J = J.*BW;

           Jref = imcrop(J, ref_rect);
           [Yr, Xr] = find(Jref(:,:,1) > ref_thr);
           RefPoints = [Xr, Yr] + double([ref_rect(1), ref_rect(2)]);
           OnePlane_refPoints = [OnePlane_refPoints;RefPoints];
        end
        refPoint = mean(OnePlane_refPoints);
        
        %一つの動画から得られる情報の保存
        All_planeparams = [All_planeparams;opt_planeparams];
        All_refPoints = [All_refPoints;refPoint];
        All_cameraPoints = [All_cameraPoints;OnePlane_cameraPoints];
        All_fvals = [All_fvals; min_fval];
        PointsCnts = [PointsCnts;size(OnePlane_cameraPoints,1)];
    end
    
    %matファイルの全ての結果の保存
    save linelaserparams.mat All_planeparams All_refPoints All_cameraPoints All_fvals PointsCnts
    
    %デバッグ：Calibration結果を図で出力
%     f = figure;
%     cnt = 0;
%     for i = 1:10
%         X = All_cameraPoints(1+cnt:cnt+PointsCnts(i),1);
%         Y = All_cameraPoints(1+cnt:cnt+PointsCnts(i),2);
%         Z = All_cameraPoints(1+cnt:cnt+PointsCnts(i),3);
%         scatter3(X,Y,Z);
%         hold on
%         graphX = linspace(min(X),max(X),500);
%         graphY = linspace(min(Y),max(Y),500);
%         [gX,gY] = meshgrid(graphX,graphY);
%         gZ = -All_planeparams(i,1)/All_planeparams(i,3)*gX-All_planeparams(i,2)/All_planeparams(i,3)*gY+1/All_planeparams(i,3);
%         mesh(gX,gY,gZ);
%         cnt = cnt + PointsCnts(i);
%     end
    
end