function linelaser_calibration_rgb()
%linelaser_calibration ラインレーザのCalibration
%   
    %.matファイルから変数群の呼び出し  
    load fishparams.mat fisheyeParams
    load setup_rgb.mat linelaser_folder laser_step squareSize laser_time_margin ...
    margin bright_r_thr ref_circle_radi ref_r_thr ref_arcwidth linelaser_file_num

    %保存用の行列
    All_planeparams = [];
    All_refPoints = [];
    All_cameraPoints = [];
    All_fvals = [];
    All_PointsCnts = [];
    All_Opt_cameraPoints = [];
    All_Opt_fvals = [];
    All_Opt_PointsCnts = [];
    OnePlane_refPoints = [];
    
    %全パターンの動画から1枚づつ取ってきてそこから参照面の円を検出
    All_centers = [];
    All_radii = [];
    for k = linelaser_file_num
        %動画から全てのFrameを出力する
        linelaser_dir = strcat(strcat(linelaser_folder, int2str(k)),'.mp4');
        vidObj = VideoReader(linelaser_dir);
        img = read(vidObj,1);
        %一つ画像を持ってきて，ハフ変換で最も有力の参照面の円を検出
        [centers, radii, metric] = imfindcircles(img,ref_circle_radi);
        if size(centers,1)~=0
            center = centers(1,:);
            radi = radii(1,:);
            All_centers = [All_centers;center];
            All_radii = [All_radii;radi];
        end
    end
    %全てのパターンから参照面の円の中心と半径の平均を取る
    ref_center = mean(All_centers,1);
    ref_radi = mean(All_radii,1);
    %円弧上のMask画像を作成する
    mask = zeros(size(img));
    mask = insertShape(mask,'circle',[ref_center ref_radi-(ref_arcwidth/2)],'LineWidth',ref_arcwidth,'Color','white');
    mask = imbinarize(mask);
    mask = uint8(mask);
    
    %レーザ動画の個数だけ行う
    for k=linelaser_file_num
        %画像群呼び出し
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
        OnePlane_refPoints = [];
        for i=1:size(detectedimgs, 4)
           J = detectedimgs(:,:,:,i);
           R = J(:,:,1)>bright_r_thr;
           J = J.*uint8(R);

           %レーザ点群取得
           rect = zeros(size(R));
           area =int16([min(imagePoints(:,:,i))-margin, max(imagePoints(:,:,i))-min(imagePoints(:,:,i))+2*margin]); % チェッカーボード周辺の矩形
           rect(max(area(2),1):min(area(2)+area(4),size(J,2)),max(area(1),1):min(area(1)+area(3),size(J,1))) = 1;
           Jtrim = J.*uint8(rect); % Trimming
           
           %輝度重心の計算(サブピクセル単位)
           BrightPoints = [];
           for r=100:1:430
               mass = 0;
               momx = 0;
               momy = 0;
               forend = uint32(3.15*2*r);
               for dt=1:forend
                   x = uint32(448+r*cos(double(dt)/r));
                   y=uint32(448+r*sin(double(dt)/r));
                   if Jtrim(y,x,1)>0
                       mass = mass + double(Jtrim(y,x,1));
                       momx = momx + double(Jtrim(y,x,1))*double(x);
                       momy = momy + double(Jtrim(y,x,1))*double(y);
                   end
               end
               if mass>0
                   BrightPoints = [BrightPoints;momx/mass,momy/mass];
               end
           end

           %レーザの輝点の画像座標->World座標系に変換
           if size(BrightPoints,2)>0
               LaserworldPoints = pointsToWorld(fisheyeParams.Intrinsics, RotMatrix(:,:,i),TransVec(i,:),BrightPoints);
               newworldPoints = [LaserworldPoints, zeros(size(LaserworldPoints,1),1)];
               %World->Camera座標系に変換
               cameraPoints = newworldPoints * RotMatrix(:,:,i) + TransVec(i,:);
               OnePlane_cameraPoints = [OnePlane_cameraPoints;cameraPoints];
           end
           %参照面にレーザ映る場所の輝点抽出
           Jref = J.* mask;
           [Yr, Xr] = find(Jref(:,:,1) > ref_r_thr);
           massref = 0;
           momxref = 0;
           momyref = 0;
           for xr = min(Xr):max(Xr)
               for yr = min(Yr):max(Yr)
                   if Jref(yr,xr,1)>ref_r_thr
                       massref = massref + double(Jref(yr,xr,1));
                       momxref = momxref + double(Jref(yr,xr,1))*double(xr);
                       momyref = momyref + double(Jref(yr,xr,1))*double(yr);
                   end
               end
           end
           if massref > 0
               RefPoints = [momxref/massref, momyref/massref];
               OnePlane_refPoints = [OnePlane_refPoints;RefPoints];
           end
        end

        %レーザ輝点群を最小二乗法で一つの平面を出力
        if k<=7 || k>=21
            norm = 0;
            maxcnt = 3;
        else
            norm = -0.001;
            maxcnt = 10;
        end
        func = @(param)calcplane_func(param, OnePlane_cameraPoints);
        min_fval = 1e+20;
        for cnt = 1:maxcnt
            x0 = -rand(1,3)*norm;%0.001
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
        opt_min_fval = 1e+20;
        for cnt = 1:maxcnt
            x0 = -rand(1,3)*norm;%0.001
            options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
            [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
            if exitflag==1&&fval<3000
                opt_planeparams = planeparams;
                break;
            elseif exitflag==1
                if opt_min_fval > fval
                    opt_min_fval = fval;
                    opt_planeparams = planeparams;
                end
            end
        end

        %参照面の輝点出力
%         OnePlane_refPoints = [];
%         for i=1:size(detectedimgs, 4)
%            J = detectedimgs(:,:,:,i);
%            Jref = J.* mask;
%            [Yr, Xr] = find(Jref(:,:,1) > ref_thr);
%            RefPoints = [Xr, Yr];
%            OnePlane_refPoints = [OnePlane_refPoints;RefPoints];
%         end
        refPoint = mean(OnePlane_refPoints);
        
        %一つの動画から得られる情報の保存
        All_planeparams = [All_planeparams;opt_planeparams];
        All_refPoints = [All_refPoints;refPoint];
        All_cameraPoints = [All_cameraPoints;OnePlane_cameraPoints];
        All_fvals = [All_fvals; min_fval];
        All_PointsCnts = [All_PointsCnts;size(OnePlane_cameraPoints,1)];
        All_Opt_cameraPoints = [All_Opt_cameraPoints;Opt_OnePlane_cameraPoints];
        All_Opt_fvals = [All_Opt_fvals; opt_min_fval];
        All_Opt_PointsCnts = [All_Opt_PointsCnts;size(Opt_OnePlane_cameraPoints,1)];
    end
    
    %matファイルの全ての結果の保存
    save linelaserparams.mat All_planeparams All_refPoints ref_center ref_radi All_cameraPoints All_fvals All_PointsCnts
    
    %デバッグ：Calibration結果を図で出力
%     All_PointsID = [0;All_PointsCnts(1)];
%     for i = 2:23
%         All_PointsID = [All_PointsID;All_PointsCnts(i)+All_PointsID(i)];
%     end
%     f = figure;
%     for i = 1:23
%         X = All_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),1);
%         Y = All_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),2);
%         Z = All_cameraPoints(1+All_PointsID(i):All_PointsID(i+1),3);
%         scatter3(X,Y,Z);
%         hold on
%         graphX = linspace(min(X),max(X),500);
%         graphY = linspace(min(Y),max(Y),500);
%         [gX,gY] = meshgrid(graphX,graphY);
%         gZ = -All_planeparams(i,1)/All_planeparams(i,3)*gX-All_planeparams(i,2)/All_planeparams(i,3)*gY+1/All_planeparams(i,3);
%         mesh(gX,gY,gZ);
%     end
    
end