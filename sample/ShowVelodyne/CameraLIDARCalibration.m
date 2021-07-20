%パラメータ設定
idnum = 8;
camparamdir = '202103151940_fisheyeparam.csv';
cameradir = 'data\calibration\Camera\';
lidardir = 'data\calibration\LIDAR\';
squareSize = 32;
lidar_cb_angles = [100 115;
                   130 145;
                   160 185;
                   185 220;
                   270 305;
                   315 355;
                   0 20;
                   10 35];
lidar_zeroanglenum = 9;

%カメラパラメータ読み取り
M = csvread(camparamdir);
mapcoeff = M(1,1:4);
imagesize = [896 896];
distortion = M(3,1:2)+1;
intrinsics = fisheyeIntrinsics(mapcoeff,imagesize,distortion);

lidarpts_oncb = [];
norm_cam2cbs = [];
for i = 8
    for k = lidar_zeroanglenum
        
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