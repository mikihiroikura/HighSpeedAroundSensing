%チェッカーボード周囲のLIDAR取得点群の空間分解能計算

Filename = 'data\evaluation\LIDAR\2021-07-20-15-23-31_Velodyne-HDL-32-Data.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
checkerRanges = [-0.7, -0.4;
                0.4, 0.8;
                -0.2, 0.1];%チェッカーボード板のある範囲を指定する

totalptcnt = 0;
for i = 1:velodyne.NumberOfFrames
   pcobj = readFrame(velodyne, i);
   id = pcobj.Location(:,:,1) > checkerRanges(1,1) & pcobj.Location(:,:,1) < checkerRanges(1,2) ...
       & pcobj.Location(:,:,2) > checkerRanges(2,1) & pcobj.Location(:,:,2) < checkerRanges(2,2) ...
       & pcobj.Location(:,:,3) > checkerRanges(3,1) & pcobj.Location(:,:,3) < checkerRanges(3,2);
   ptcnt = sum(sum(id));
   totalptcnt = totalptcnt + ptcnt;
end
%全フレームでのチェッカーボード板の範囲内の点数の平均
meanptcnt = totalptcnt / velodyne.NumberOfFrames;

%空間分解能の計算
meanptcnt/((checkerRanges(2,2)-checkerRanges(2,1)) * (checkerRanges(3,2)-checkerRanges(3,1))) / 10000

%計測精度の評価
pcobj = readFrame(velodyne, int8(velodyne.NumberOfFrames/2));
X = pcobj.Location(:,:,1);
Y = pcobj.Location(:,:,2);
Z = pcobj.Location(:,:,3);
id = pcobj.Location(:,:,1) > checkerRanges(1,1) & pcobj.Location(:,:,1) < checkerRanges(1,2) & ...
pcobj.Location(:,:,2) > checkerRanges(2,1) & pcobj.Location(:,:,2) < checkerRanges(2,2) & ...
pcobj.Location(:,:,3) > checkerRanges(3,1) & pcobj.Location(:,:,3) < checkerRanges(3,2);
Checker_cameraPoints = double([X(id), Y(id),Z(id)]);
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
%平面との距離[mm]
dists = (opt_planeparams(1).*Checker_cameraPoints(:,1) + opt_planeparams(2).*Checker_cameraPoints(:,2) ...
    + opt_planeparams(3).*Checker_cameraPoints(:,3) - 1)./norm(opt_planeparams)*1000;
mean(dists)
std(dists)

%% LIDAR点群と平面の出力
f = figure;
scatter3(Checker_cameraPoints(:,1),Checker_cameraPoints(:,2),Checker_cameraPoints(:,3));
hold on
graphX = linspace(min(Checker_cameraPoints(:,1)),max(Checker_cameraPoints(:,1)),50);
graphY = linspace(min(Checker_cameraPoints(:,2)),max(Checker_cameraPoints(:,2)),50);
[gX,gY] = meshgrid(graphX,graphY);
gZ = -opt_planeparams(1)/opt_planeparams(3)*gX-opt_planeparams(2)/opt_planeparams(3)*gY+1/opt_planeparams(3);
s = mesh(gX,gY,gZ);
daspect([1 1 1]);