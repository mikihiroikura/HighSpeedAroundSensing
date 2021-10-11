%チェッカーボード周囲のLIDAR取得点群の空間分解能計算

Filename = 'data\evaluation\LIDAR\2021-08-03-21-02-32_Velodyne-HDL-32-Data_thesiscand.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
checkerRanges = [-0.55, -0.4;
                0.6, 0.8;
                -0.1, 0.1];%チェッカーボード板のある範囲を指定する

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
spatioresol = meanptcnt/((checkerRanges(2,2)-checkerRanges(2,1)) * (checkerRanges(3,2)-checkerRanges(3,1))) / 10000;

%% 計測精度の評価
frameid = randperm(velodyne.NumberOfFrames);
Checker_LIDARPoints = [];
for readid = frameid
    pcobj = readFrame(velodyne, readid);
    X = pcobj.Location(:,:,1);
    Y = pcobj.Location(:,:,2);
    Z = pcobj.Location(:,:,3);
    id = pcobj.Location(:,:,1) > checkerRanges(1,1) & pcobj.Location(:,:,1) < checkerRanges(1,2) & ...
    pcobj.Location(:,:,2) > checkerRanges(2,1) & pcobj.Location(:,:,2) < checkerRanges(2,2) & ...
    pcobj.Location(:,:,3) > checkerRanges(3,1) & pcobj.Location(:,:,3) < checkerRanges(3,2);
    Checker_LIDARPoints = [Checker_LIDARPoints; double([X(id), Y(id),Z(id)] * 1000)];
end

%平面フィッティング
if size(Checker_LIDARPoints,1)<10000
    psize = size(Checker_LIDARPoints,1);
else
    psize = 10000;
end
opt_planeparams = planefitting_func(Checker_LIDARPoints(1:psize,:), 0);

%平面との距離[mm]
dists = (opt_planeparams(1).*Checker_LIDARPoints(:,1) + opt_planeparams(2).*Checker_LIDARPoints(:,2) ...
    + opt_planeparams(3).*Checker_LIDARPoints(:,3) - 1)./norm(opt_planeparams);
mean(dists)
std(dists)
spatioresol
%% LIDAR点群と平面の出力
f = figure;
colormap jet;
scatter3(Checker_LIDARPoints(:,1),Checker_LIDARPoints(:,2),Checker_LIDARPoints(:,3),[], dists);
hold on
graphX = linspace(min(Checker_LIDARPoints(:,1)),max(Checker_LIDARPoints(:,1)),50);
graphY = linspace(min(Checker_LIDARPoints(:,2)),max(Checker_LIDARPoints(:,2)),50);
[gX,gY] = meshgrid(graphX,graphY);
gZ = -opt_planeparams(1)/opt_planeparams(3)*gX-opt_planeparams(2)/opt_planeparams(3)*gY+1/opt_planeparams(3);
s = mesh(gX,gY,gZ);
daspect([1 1 1]);
xlim([checkerRanges(1,1) checkerRanges(1,2)]*1000);
zlim([checkerRanges(3,1) checkerRanges(3,2)]*1000);

%% MDPI用の図の作成
% 点群提示範囲を決める
showRanges = [-0.55, -0.4;
                0.63, 0.77;
                -0.1, 0.1];%チェッカーボード板のある範囲を指定する
% 平面の法線ベクトルがX軸になるような平面に変換する
planenorm = opt_planeparams/norm(opt_planeparams);
inplane_z0 = [planenorm(2) -planenorm(1) 0]/norm(planenorm(1:2));
inplane_other = cross(planenorm,inplane_z0);
R_orig2view = inv([planenorm;inplane_z0;inplane_other]);

% Scan回数の点群を出力する
scannum = 20;
scanid = randperm(velodyne.NumberOfFrames,scannum);
f = figure;
colormap jet;
Clim = [-30 30];
for j = scanid
    pcobj = readFrame(velodyne, j);
    X = pcobj.Location(:,:,1);
    Y = pcobj.Location(:,:,2);
    Z = pcobj.Location(:,:,3);
    id = pcobj.Location(:,:,1) > showRanges(1,1) & pcobj.Location(:,:,1) < showRanges(1,2) & ...
    pcobj.Location(:,:,2) > showRanges(2,1) & pcobj.Location(:,:,2) < showRanges(2,2) & ...
    pcobj.Location(:,:,3) > showRanges(3,1) & pcobj.Location(:,:,3) < showRanges(3,2);
    Checker_LIDARPoint_oneframe = double([X(id), Y(id),Z(id)] * 1000) * R_orig2view;
    transplanenorm = opt_planeparams * R_orig2view;
    dist_oneframe = -(transplanenorm(1).*Checker_LIDARPoint_oneframe(:,1) + transplanenorm(2).*Checker_LIDARPoint_oneframe(:,2) ...
    + transplanenorm(3).*Checker_LIDARPoint_oneframe(:,3) - 1)./norm(transplanenorm);
    scatter3(Checker_LIDARPoint_oneframe(:,1), Checker_LIDARPoint_oneframe(:,2), Checker_LIDARPoint_oneframe(:,3), ...
    [], dist_oneframe,'o','filled');
    hold on
%     if Clim(1)>min(dist_oneframe)
%         Clim(1) = min(dist_oneframe);
%     end
%     if Clim(2)<max(dist_oneframe)
%         Clim(2) = max(dist_oneframe);
%     end
end
daspect([1 1 1]);
graphZ = linspace(-200,400,13);
graphY = linspace(250,850,13);
[gZ,gY] = meshgrid(graphZ,graphY);
gX = -transplanenorm(2)/transplanenorm(1)*gY-transplanenorm(3)/transplanenorm(1)*gZ+1/transplanenorm(1);
s = mesh(gX,gY,gZ,'EdgeColor',[0 0 0], 'FaceColor', 'none', 'FaceAlpha', 1);
daspect([1 1 1]);
grid off;
ax = gca;
ax.CLim = Clim;
ax.XAxis.Visible = 'off';
ax.YAxis.Visible = 'off';
ax.ZAxis.Visible = 'off';
margin = 20;
ylim([450-margin 650+margin]);
zlim([-75-margin 175+margin]);
view(-135,20);
%% 点群を座標変換
print(gcf,'-painters','a','-dpdf');