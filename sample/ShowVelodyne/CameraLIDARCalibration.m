%パラメータ設定
idnum = 8;
camparamdir = '202103151940_fisheyeparam.csv';
cameradir = 'data\calibration\Camera\';
lidardir = 'data\calibration\LIDAR\';
squareSize = 28.3;
lidar_cb_angles = [120 135;
                   170 185;
                   278 288;
                   185 220;
                   320 347;
                   315 355;
                   7 23;
                   113 123];
lidar_anglenums = [9 9 7 9 9 9 9 9];
lidar_zeroanglenum = 9;

%カメラパラメータ読み取り
M = csvread(camparamdir);
mapcoeff = M(1,1:4);
imagesize = [896 896];
distortion = M(3,1:2)+1;
intrinsics = fisheyeIntrinsics(mapcoeff,imagesize,distortion);

lidarpts_oncb = [];
norm_cam2cbs = [];
for i = [1 2 3 5 7 8]
    for k = lidar_anglenums(i)
        
        %画像からCB検出
        imgname = strcat(cameradir, strcat(num2str(i), '.png'));
        [imagePoints, boardSize] = detectCheckerboardPoints(imgname);
        worldPoints = generateCheckerboardPoints(boardSize, squareSize);
        [R, t] = extrinsics(imagePoints, worldPoints, intrinsics);
        norm_cam2cb = R(3,:) * dot(R(3,:),t);

        %LIDARのCB上の点群取得
        lidarname = strcat(lidardir, strcat(num2str(i), '.pcap'));
        velodyne = velodyneFileReader(lidarname,'HDL32E');
        pcobj = readFrame(velodyne, int8(velodyne.NumberOfFrames/2));
        theta = atan2(pcobj.Location(k,:,2),pcobj.Location(k,:,1)) * 180 /pi;
        for j = 1:size(theta,2)
            if theta(j) < 0
                theta(j) = theta(j) + 360;
            end
        end
        id = theta > lidar_cb_angles(i,1) & theta < lidar_cb_angles(i,2);
        lidarpts_oncb = [lidarpts_oncb;pcobj.Location(k,id,1).',pcobj.Location(k,id,2).',pcobj.Location(k,id,3).'];

        X = repmat(norm_cam2cb, sum(id), 1);
        norm_cam2cbs = [norm_cam2cbs; X];
    end
end

%Levenberg-marquardt法によるCam2Lidarの位置姿勢計算
%ここ虚数になってしまうので最適化できない．．．
px = double(lidarpts_oncb(:,1));%単位はm
py = double(lidarpts_oncb(:,2));%単位はm
pz = double(lidarpts_oncb(:,3));%単位はm
n1 = norm_cam2cbs(:,1)*0.001;%単位をmに変換
n2 = norm_cam2cbs(:,2)*0.001;%単位をmに変換
n3 = norm_cam2cbs(:,3)*0.001;%単位をmに変換
fun = @(x) ...
n1.*((px-x(4)).*(1-2*(x(2)^2+x(3)^2)) +(py-x(5)).*2*(x(1)*x(2)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(3))+(pz-x(6))*2*(x(1)*x(3)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(2)))+ ...
n2.*((px-x(4)).*2*(x(1)*x(2)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(3)) +(py-x(5)).*(1-2*(x(1)^2+x(3)^2))+(pz-x(6))*2*(x(2)*x(3)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(1)))+ ...
n3.*((px-x(4)).*2*(x(1)*x(3)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(2)) +(py-x(5)).*2*(x(2)*x(3)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(1))+(pz-x(6))*(1-2*(x(1)^2+x(2)^2)))- n1.^2 - n2.^2 - n3.^2;
x0 = [0 0 0 0 0 0];
options = optimoptions(@lsqnonlin,'Display','iter','Algorithm','levenberg-marquardt', 'MaxFunctionEvaluations', 2000);
x = lsqnonlin(fun, x0, [-1 -1 -1 -10 -10 -10], [1 1 1 10 10 10], options);

Rans = [(1-2*(x(2)^2+x(3)^2)) 2*(x(1)*x(2)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(3)) 2*(x(1)*x(3)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(2));
        2*(x(1)*x(2)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(3)) (1-2*(x(1)^2+x(3)^2)) 2*(x(2)*x(3)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(1));
        2*(x(1)*x(3)+(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(2)) 2*(x(2)*x(3)-(1-(x(1)^2+x(2)^2+x(3)^2))^0.5*x(1)) (1-2*(x(1)^2+x(2)^2))];
    
    
%最適化
ns = norm_cam2cbs * 0.001;
pl_oncb = double(lidarpts_oncb);
func = @(param)ReprojectCam2LIDAR(param, ns, pl_oncb);
x0 = [0 0 0 0 0 0];
options = optimoptions(@lsqnonlin,'Display','iter','Algorithm','levenberg-marquardt', 'MaxFunctionEvaluations', 2000);
x = lsqnonlin(func, x0, [-pi -pi -pi -10 -10 -10], [pi pi pi 10 10 10], options);


%% 3つのCBの平面の式から回転行列求める
planenum = [2 7 8];
cbnorm_atcam=[];
cbnorm_atlidar = [];
checkXmins = [-0.8, -10000];
checkXmaxs = [-0.6, -700];
checkYmins = [-0.08,-220];
checkYmaxs = [0.1, 30];
checkZmins = [-0.1, 280];
checkZmaxs = [0.15, 480];
for v = 1:3
    j = planenum(v);
    imgname = strcat(cameradir, strcat(num2str(j), '.png'));
    [imagePoints, boardSize] = detectCheckerboardPoints(imgname);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    [R, t] = extrinsics(imagePoints, worldPoints, intrinsics);

    %CBの点群をCamera座標系に変換
    worldPts = [worldPoints,zeros(size(worldPoints,1),1)];
    Checker_cameraPoints = worldPts * R + t;


    maxcnt = 3;
    norms = -0.001;
    %平面最適化で式を計算
    func = @(param)calcplane_func(param, Checker_cameraPoints);
    min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norms;%0.001
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
    dist = abs(1-(opt_planeparams(1).*Checker_cameraPoints(:,1)+opt_planeparams(2) ...
            .*Checker_cameraPoints(:,2)+opt_planeparams(3).*Checker_cameraPoints(:,3))) ...
            ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    Opt_Checker_cameraPoints = Checker_cameraPoints(dist<10,:);
    %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
    func = @(param)calcplane_func(param, Opt_Checker_cameraPoints);
    opt_min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norms;%0.001
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
    cbnorm_atcam = [cbnorm_atcam;opt_planeparams];
    
    lidarname = strcat(lidardir, strcat(num2str(j), '.pcap'));
    velodyne = velodyneFileReader(lidarname,'HDL32E');
    pcobj = readFrame(velodyne, int8(velodyne.NumberOfFrames/2));
    X = pcobj.Location(:,:,1);
    Y = pcobj.Location(:,:,2);
    Z = pcobj.Location(:,:,3);
    cb_area = [checkXmins(v) checkXmaxs(v);
            checkYmins(v) checkYmaxs(v);
            checkZmins(v) checkZmaxs(v)];%CBの範囲指定
    id = pcobj.Location(:,:,1) > cb_area(1,1) & pcobj.Location(:,:,1) < cb_area(1,2) & ...
    pcobj.Location(:,:,2) > cb_area(2,1) & pcobj.Location(:,:,2) < cb_area(2,2) & ...
    pcobj.Location(:,:,3) > cb_area(3,1) & pcobj.Location(:,:,3) < cb_area(3,2);

    Checker_cameraPoints = double([X(id), Y(id),Z(id)]);
    maxcnt = 3;
    norms = -0.00;
    %平面最適化で式を計算
    func = @(param)calcplane_func(param, Checker_cameraPoints);
    min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norms;%0.001
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
    dist = abs(1-(opt_planeparams(1).*Checker_cameraPoints(:,1)+opt_planeparams(2) ...
            .*Checker_cameraPoints(:,2)+opt_planeparams(3).*Checker_cameraPoints(:,3))) ...
            ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    Opt_Checker_cameraPoints = Checker_cameraPoints(dist<10,:);
    %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
    func = @(param)calcplane_func(param, Opt_Checker_cameraPoints);
    opt_min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norms;%0.001
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
end