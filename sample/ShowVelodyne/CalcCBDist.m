%カメラパラメータ読み取り
camparamdir = '202103151940_fisheyeparam.csv';
M = csvread(camparamdir);
mapcoeff = M(1,1:4);
imagesize = [896 896];
distortion = M(3,1:2)+1;
intrinsics = fisheyeIntrinsics(mapcoeff,imagesize,distortion);

%画像からCB検出
imgname = 'data\202107201431_img00.png';
squareSize = 32;

[imagePoints, boardSize] = detectCheckerboardPoints(imgname);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[R, t] = extrinsics(imagePoints, worldPoints, intrinsics);
norm_cam2cb = R(3,:) * dot(R(3,:),t);

%CBの点群をCamera座標系に変換
worldPts = [worldPoints,zeros(size(worldPoints,1),1)];
camPts = worldPts * R + t;


maxcnt = 3;
norms = -0.001;
%平面最適化で式を計算
func = @(param)calcplane_func(param, camPts);
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
dist = abs(1-(opt_planeparams(1).*camPts(:,1)+opt_planeparams(2) ...
        .*camPts(:,2)+opt_planeparams(3).*camPts(:,3))) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
Opt_Checker_cameraPoints = camPts(dist<10,:);
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

dist_fromcam = (1-(opt_planeparams(1)*0+opt_planeparams(2) ...
        *0+opt_planeparams(3)*0)) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    
f = figure;
scatter3(camPts(:,1),camPts(:,2),camPts(:,3));
hold on
graphX = linspace(min(camPts(:,1)),max(camPts(:,1)),50);
graphY = linspace(min(camPts(:,2)),max(camPts(:,2)),50);
[gX,gY] = meshgrid(graphX,graphY);
gZ = -opt_planeparams(1)/opt_planeparams(3)*gX-opt_planeparams(2)/opt_planeparams(3)*gY+1/opt_planeparams(3);
s = mesh(gX,gY,gZ);
daspect([1 1 1]);