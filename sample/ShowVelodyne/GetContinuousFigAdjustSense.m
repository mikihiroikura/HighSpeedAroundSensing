%%% AdjustSenseのCSVファイルから指定連続Indexの点群を図で表示する
%% ファイルの指定，計測対象範囲指定
csvname = 'data\evaluation\Camera\211011170805_LSM_result_recipro_thesiscand.csv';

checkXmins = [-1000, 100000];
checkXmaxs = [1000, 10000];
checkYmins = [-10000,-220];
checkYmaxs = [10000, 30];
checkZmins = [380, 280];
checkZmaxs = [450, 480];
checkid = 1;
checkXmin = checkXmins(checkid);
checkXmax = checkXmaxs(checkid);
checkYmin = checkYmins(checkid);
checkYmax = checkYmaxs(checkid);
checkZmin = checkZmins(checkid);
checkZmax = checkZmaxs(checkid);

%% CSVの読み取り
M =csvread(csvname);
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
ref_center = [445.068326,404.6309];
LSM_rotdir = M(1:4:end,2);
LSM_detectedenablefig = M(1:4:end,3);
LSM_objdetectedflg = M(1:4:end,4);
LSM_reciprocntdown = M(1:4:end,5);
LSM_alertcnt = M(1:4:end,6);
LSM_dangercnt = M(1:4:end,7);
LSM_rpms = M(1:4:end,8);
LSM_laserplane_nml = M(1:4:end,9:11);
refpts = M(1:4:end,12:13);
LSM_rotmode = M(1:4:end,14);
%参照面の輝点が参照面中央に対してどの方向にあるか
dirs = refpts - ref_center;
rads = atan2(dirs(:,2),dirs(:,1));

%% グラフの指定
rangeon = 1;%ON: 1 OFF: 0
xranges = [-700 500];
yranges = [-1000 -700];
zranges = [300 700];

%デバッグ:時空間分解能計算
Modes = [];
difftime = [];
Pointcnt = [];
Ranges = [];
Timeold = 0;
drads = 0;

%出力図の相フレーム数の指定
%0ならば自動で出力の時間を変える
%それ以外なら指定の出力フレーム数
framenum = 0;

%点群の距離ごとに色を指定
Ac = 2000;
Dc = 1500;
dist = (Xs.^2 + Ys.^2).^0.5;
ColorMat = int8(ones(size(dist)));
ColorMat = ColorMat + int8(dist > Ac);
ColorMat = ColorMat + int8(dist > Dc);
colormap_mat = [1 0 0
            0 1 0
            0 0 1];

%% 動画と画像の保存先指定
format = 'yyyymmddHHMM';
imgfolder = strcat('D:/Github_output/HighSpeedAroundSensing/ShowVelodyne/ContinuousFigAdjustSense/',datestr(now,format));
mkdir(imgfolder);
v = VideoWriter(strcat(imgfolder,'/LSM_result'),'MPEG-4');
open(v)

%% グラフ描画
dirid = 1;
hold off
idold = 1;
for i=2:size(Times,1)
    if abs(rads(i)-rads(i-1)) > 5
        drads =drads - abs(rads(i)-rads(i-1)) + 2 * pi;
    else
        drads =drads + abs(rads(i)-rads(i-1));
    end
    %ここから計測モードによってどれだけHold onするか決める
    if framenum == 0
        if LSM_rotmode(i) == 1.0%局所領域計測時
            if LSM_rotdir(i) ~= LSM_rotdir(i-1)%回転方向が変化したとき
                Xsp = Xs(idold:i,:);
                Ysp = Ys(idold:i,:);
                Zsp = Zs(idold:i,:);
                ColorMatsp = ColorMat(idold:i,:);
                scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap_mat(ColorMatsp(Xsp~=0),:), 'o', 'filled');
                if rangeon == 1
                    xlim(xranges);
                    ylim(yranges);
                    zlim(zranges);
                else
                    xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                    ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                    zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                end
                daspect([1 1 1]);
                view(-10,10);
                ax = gca;
                ax.ZDir = 'reverse';
                ax.YDir = 'reverse';
                ax.XAxis.FontSize = 15;
                ax.ZAxis.FontSize = 15;
                ax.YAxis.FontSize = 15;
                title(['Time[s]: ',num2str(Times(i,1),'%.3f')], 'FontSize', 20);
                frame =getframe(gcf);
                writeVideo(v,frame);
                imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid))));
                print(gcf,'-painters',imgfile,'-dpdf');
                dirid = dirid +1;
                hold off
                idold = i;
                %時空間分解能計算
                Modes = [Modes;"ReciprLR"];
                Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                difftime =[difftime;Times(i)-Timeold];
                Ranges = [Ranges; drads];
                drads = 0;
                Timeold = Times(i);
            end
        else%全周計測時
            if LSM_rotdir(i) == 0%右回転
                if rads(i) > 0 && rads(i-1) < 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    ColorMatsp = ColorMat(idold:i,:);
                    scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap_mat(ColorMatsp(Xsp~=0),:), 'o', 'filled');
                    if rangeon == 1
                        xlim(xranges);
                        ylim(yranges);
                        zlim(zranges);
                    else
                        xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                        ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                        zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                    end
                    daspect([1 1 1]);
                    view(-10,10);
                    ax = gca;
                    ax.ZDir = 'reverse';
                    ax.YDir = 'reverse';
                    ax.XAxis.FontSize = 15;
                    ax.ZAxis.FontSize = 15;
                    ax.YAxis.FontSize = 15;
                    title(['Time[s]: ',num2str(Times(i,1),'%.3f')], 'FontSize', 20);
                    frame =getframe(gcf);
                    writeVideo(v,frame);
                    imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid))));
                    print(gcf,'-painters',imgfile,'-dpdf');
                    dirid = dirid +1;
                    hold off
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-R"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                end
            else%左回転
                if rads(i) < 0 && rads(i-1) > 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    ColorMatsp = ColorMat(idold:i,:);
                    scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap_mat(ColorMatsp(Xsp~=0),:), 'o', 'filled');
                    if rangeon == 1
                        xlim(xranges);
                        ylim(yranges);
                        zlim(zranges);
                    else
                        xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                        ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                        zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                    end
                    daspect([1 1 1]);
                    view(-10,10);
                    ax = gca;
                    ax.ZDir = 'reverse';
                    ax.YDir = 'reverse';
                    ax.XAxis.FontSize = 15;
                    ax.ZAxis.FontSize = 15;
                    ax.YAxis.FontSize = 15;
                    title(['Time[s]: ',num2str(Times(i,1),'%.3f')], 'FontSize', 20);
                    frame =getframe(gcf);
                    writeVideo(v,frame);
                    imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid))));
                    print(gcf,'-painters',imgfile,'-dpdf');
                    dirid = dirid +1;
                    hold off
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-L"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                end
            end
        end
    else
        if mod(i,framenum) == 0
            Xsp = Xs(idold:i,:);
            Ysp = Ys(idold:i,:);
            Zsp = Zs(idold:i,:);
            ColorMatsp = ColorMat(idold:i,:);
            scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap_mat(ColorMatsp(Xsp~=0),:), 'o', 'filled');
            if rangeon == 1
                xlim(xranges);
                ylim(yranges);
                zlim(zranges);
            else
                xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
            end
            daspect([1 1 1]);
            view(-10,10);
            ax = gca;
            ax.ZDir = 'reverse';
            ax.YDir = 'reverse';
            ax.XAxis.FontSize = 15;
            ax.ZAxis.FontSize = 15;
            ax.YAxis.FontSize = 15;
            title(['Time[s]: ',num2str(Times(i,1),'%.3f')], 'FontSize', 20);
            frame =getframe(gcf);
            writeVideo(v,frame);
            imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid))));
            print(gcf,'-painters',imgfile,'-dpdf');
            dirid = dirid +1;
            hold off
            idold = i;
        end
    end

end
close(v);
%% Start indexの指定&表示範囲の指定
startid = 21507;
boardRanges = [-300 300;
               -900 -750;
               300 700];
%% 検索範囲指定:id154
setid = 504;
index = 21457:21507;
checkerRanges = [-25 125;
                -900 -750;
                350 600];%チェッカーボード板のある範囲を指定する
%% 検索範囲指定:id155
setid = 505;
index = 21507:21555;
checkerRanges = [-50 100;
                -900 -750;
                350 600];%チェッカーボード板のある範囲を指定する
%% 検索範囲指定:id156
setid = 506;
index = 21555:21606;
checkerRanges = [-75 75;
                -900 -750;
                350 600];%チェッカーボード板のある範囲を指定する
%% 検索範囲指定:id157
setid = 507;
index = 21606:21652;
checkerRanges = [-100 50;
                -900 -750;
                350 600];%チェッカーボード板のある範囲を指定する
%% 検索範囲指定:id158
setid = 508;
index = 21652:21700;
checkerRanges = [-125 25;
                -900 -750;
                350 600];%チェッカーボード板のある範囲を指定する
%% 全ての表示
f = figure;
Xsp = Xs(index,:);
Ysp = Ys(index,:);
Zsp = Zs(index,:);
ColorMatsp = ColorMat(index,:);
scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap_mat(ColorMatsp(Xsp~=0),:), 'o', 'filled');
xlim(boardRanges(1,:));
ylim(boardRanges(2,:));
zlim(boardRanges(3,:));
daspect([1 1 1]);
view(-10,10);
ax = gca;
ax.ZDir = 'reverse';
ax.YDir = 'reverse';
ax.XAxis.FontSize = 15;
ax.ZAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
title(['Time[s]: ',num2str(Times(index(end))-Times(startid),'%.3f')], 'FontSize', 20);

%% Checkerboard上の点群表示
onchecker_id = (Xsp~=0) & (Xsp > checkerRanges(1,1)) & (Xsp < checkerRanges(1,2)) ...
    & (Ysp > checkerRanges(2,1)) & (Ysp < checkerRanges(2,2)) ...
    & (Zsp > checkerRanges(3,1)) & (Zsp < checkerRanges(3,2));
Xfit = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
Yfit = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
Zfit = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
f = figure;
scatter3(Xfit,Yfit,Zfit);
daspect([1 1 1]);

%% 平面フィッティング
%Checkerboard上の点群をフィッティング
opt_planeparams = planefitting_func(double([Xfit,Yfit,Zfit]),0);
%計測された全ての点群をフィッティングした平面との距離を計算
show_id = (Xsp~=0);
Xshow = reshape(Xsp(show_id), [size(Xsp(show_id),1), 1]);
Yshow = reshape(Ysp(show_id), [size(Ysp(show_id),1), 1]);
Zshow = reshape(Zsp(show_id), [size(Zsp(show_id),1), 1]);
%計測された全ての点群と平面との距離[mm]
dists = (opt_planeparams(1).*double(Xshow) + opt_planeparams(2).*double(Yshow) ...
    + opt_planeparams(3).*double(Zshow) - 1)./norm(opt_planeparams);
mean(dists)
std(dists)
%フィッティングに用いた点群と平面との距離[mm]
dists_oncb = (opt_planeparams(1).*double(Xfit) + opt_planeparams(2).*double(Yfit) ...
    + opt_planeparams(3).*double(Zfit) - 1)./norm(opt_planeparams);
mean(dists_oncb)
std(dists_oncb)

%% 点群表示&フィッティング平面との誤差表示
f = figure;
colormap jet;
scatter3(Xshow,Yshow,Zshow,[], dists,'o','filled');
daspect([1 1 1]);
xlim(boardRanges(1,:));
ylim(boardRanges(2,:));
zlim(boardRanges(3,:));
view(-10,10);
Clim = [-30 30];
ax = gca;
ax.ZDir = 'reverse';
ax.YDir = 'reverse';
ax.XAxis.FontSize = 15;
ax.ZAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
ax.XAxis.Visible = 'off';
ax.YAxis.Visible = 'off';
ax.ZAxis.Visible = 'off';
ax.CLim = Clim;
title(['Time[s]: ',num2str(Times(index(end))-Times(startid),'%.3f')], 'FontSize', 20);
%% 点群をPDFに保存
imgfile = strcat(imgfolder,strcat('/planefitted_frame_',strcat(sprintf("%03d",setid))));
print(gcf,'-painters',imgfile,'-dpdf');