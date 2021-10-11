%%% VelodyneのPCAPファイルから指定連続Indexの点群を図で表示する
%% ファイルの指定,ボードの範囲指定,Index指定

Filename = 'data\evaluation\LIDAR\2021-10-11-16-46-11_Velodyne-HDL-32-Data_thesiscand.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
boardRanges = [-0.9 -0.75;
                0.1 0.7;
                -0.17 0.23];%図を表示する範囲を指定
%% 動画の保存先
%動画と画像の保存先指定
format = 'yyyymmddHHMM';
imgfolder = strcat('D:/Github_output/HighSpeedAroundSensing/ShowVelodyne/ContinuousFigVelodyne/',datestr(now,format));
mkdir(imgfolder);
%% 開始Indexの指定
startid = 133;
%% 平面指定Index133
% Checkerboardがある範囲に絞って平面フィッティング
index = 133;
checkerRanges = [-0.85 -0.78;
                0.35 0.55;
                -0.15 0.15];%チェッカーボード板のある範囲を指定する
%% 平面指定Index134
% Checkerboardがある範囲に絞って平面フィッティング
index = 134;
checkerRanges = [-0.85 -0.78;
                0.30 0.50;
                -0.15 0.15];%チェッカーボード板のある範囲を指定する
%% 平面指定Index135
% Checkerboardがある範囲に絞って平面フィッティング
index = 135;
checkerRanges = [-0.85 -0.78;
                0.25 0.45;
                -0.15 0.15];%チェッカーボード板のある範囲を指定する
%% 全ての点群表示
f = figure;
pcobj = readFrame(velodyne, index);
X = pcobj.Location(:,:,1);
Y = pcobj.Location(:,:,2);
Z = pcobj.Location(:,:,3);
id_show = pcobj.Location(:,:,1) > boardRanges(1,1) & pcobj.Location(:,:,1) < boardRanges(1,2) & ...
pcobj.Location(:,:,2) > boardRanges(2,1) & pcobj.Location(:,:,2) < boardRanges(2,2) & ...
pcobj.Location(:,:,3) > boardRanges(3,1) & pcobj.Location(:,:,3) < boardRanges(3,2);
scatter3(X(id_show),Y(id_show),Z(id_show));
daspect([1 1 1]);
xlim(boardRanges(1,:));
ylim(boardRanges(2,:));
zlim(boardRanges(3,:));
view(80,10);


%% フィッティング平面上の点群表示
pcobj = readFrame(velodyne, index);
X = pcobj.Location(:,:,1);
Y = pcobj.Location(:,:,2);
Z = pcobj.Location(:,:,3);
id_oncb = pcobj.Location(:,:,1) > checkerRanges(1,1) & pcobj.Location(:,:,1) < checkerRanges(1,2) & ...
pcobj.Location(:,:,2) > checkerRanges(2,1) & pcobj.Location(:,:,2) < checkerRanges(2,2) & ...
pcobj.Location(:,:,3) > checkerRanges(3,1) & pcobj.Location(:,:,3) < checkerRanges(3,2);
f = figure;
scatter3(X(id_oncb),Y(id_oncb),Z(id_oncb));
daspect([1 1 1]);

%% 平面フィッティング
%Checkerboard上の点群をフィッティング
opt_planeparams = planefitting_func(double([X(id_oncb),Y(id_oncb),Z(id_oncb)]), 0);
%計測された全ての点群をフィッティングした平面との距離を計算
dists = (opt_planeparams(1).*double(X(id_show)) + opt_planeparams(2).*double(Y(id_show)) ...
    + opt_planeparams(3).*double(Z(id_show)) - 1)./norm(opt_planeparams);
mean(dists)
std(dists)
%フィッティングに用いた点群と平面との距離[mm]
dists_oncb = (opt_planeparams(1).*double(X(id_oncb)) + opt_planeparams(2).*double(Y(id_oncb)) ...
    + opt_planeparams(3).*double(Z(id_oncb)) - 1)./norm(opt_planeparams);
mean(dists_oncb)
std(dists_oncb)*1000

%% LIDAR点群表示&フィッティング平面との誤差表示
f = figure;
colormap jet;
scatter3(X(id_show)*1000,Y(id_show)*1000,Z(id_show)*1000,[], dists*1000,'o','filled');
daspect([1 1 1]);
xlim(boardRanges(1,:)*1000);
ylim(boardRanges(2,:)*1000);
zlim(boardRanges(3,:));
%zlim([-200 200]);きれいに表示するために自分で設定
view(80,10);
Clim = [-30 30];
ax = gca;
ax.XAxis.FontSize = 15;
ax.ZAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
ax.XAxis.Visible = 'off';
ax.YAxis.Visible = 'off';
ax.ZAxis.Visible = 'off';
ax.CLim = Clim;
title(['Time[s]: ',num2str((index-startid)*0.1,'%.3f')], 'FontSize', 20);
%% Colorbarの設置
c = colorbar('southoutside');
ax.CLim = Clim;
c.FontSize = 12;
c.Label.String = '[mm]';
c.Label.Position = [Clim(2)+2.5 -0.25 0];
%% 点群をPDFに保存
imgfile = strcat(imgfolder,strcat('/planefitted_frame_',strcat(sprintf("%03d",index))));
print(gcf,'-painters',imgfile,'-dpdf');