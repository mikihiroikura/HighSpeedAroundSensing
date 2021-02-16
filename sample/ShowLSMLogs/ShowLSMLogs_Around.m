%CSVの読み取り
M =csvread('csvs/210215192102_LSM_result_demo.csv');
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
ref_center = [446.34703,401.28674];
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
%グラフの軸の範囲の指定
rangeon = 1;
xranges = [-2000 2000];
yranges = [-2500 2000];
zranges = [0 1000];

%点群の距離ごとに色を指定
Ac = 1500;
Dc = 1000;
dist = (Xs.^2 + Ys.^2).^0.5;
ColorMat = int8(ones(size(dist)));
ColorMat = ColorMat + int8(dist > Ac);
ColorMat = ColorMat + int8(dist > Dc);
colormap = [1 0 0
            0 1 0
            0 0 1];
        
%参照面の輝点が参照面中央に対してどの方向にあるか
dirs = refpts - ref_center;
rads = atan2(dirs(:,2),dirs(:,1));

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


%動画と画像の保存先指定
format = 'yyyymmddHHMM';
imgfolder = strcat('D:/Github_output/HighSpeedAroundSensing/ShowLSMLogs/',datestr(now,format));
mkdir(imgfolder);
v = VideoWriter(strcat(imgfolder,'/LSM_result'),'MPEG-4');
open(v)
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
                scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap(ColorMatsp(Xsp~=0),:));
                if rangeon == 1
                    xlim(xranges);
                    ylim(yranges);
                    zlim(zranges);
                else
                    xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                    ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                    zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                end
                title(['Time[s]: ',num2str(Times(i,1))]);
                daspect([1 1 1]);
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
                    scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap(ColorMatsp(Xsp~=0),:));
                    if rangeon == 1
                        xlim(xranges);
                        ylim(yranges);
                        zlim(zranges);
                    else
                        xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                        ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                        zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                    end
                    title(['Time[s]: ',num2str(Times(i,1))]);
                    daspect([1 1 1]);
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
                    scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap(ColorMatsp(Xsp~=0),:));
                    if rangeon == 1
                        xlim(xranges);
                        ylim(yranges);
                        zlim(zranges);
                    else
                        xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                        ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                        zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
                    end
                    title(['Time[s]: ',num2str(Times(i,1))]);
                    daspect([1 1 1]);
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
            scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap(ColorMatsp(Xsp~=0),:));
            if rangeon == 1
                xlim(xranges);
                ylim(yranges);
                zlim(zranges);
            else
                xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
                ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
                zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
            end
            title(['Time[s]: ',num2str(Times(i,1))]);
            daspect([1 1 1]);
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

%デバッグ
% idold = 1800;
% i= 1900;
% f = figure;
% Xsp = Xs(idold:i,:);
% Ysp = Ys(idold:i,:);
% Zsp = Zs(idold:i,:);
% ColorMatsp = ColorMat(idold:i,:);
% scatter3(Xsp(Xsp~=0),Ysp(Xsp~=0),Zsp(Xsp~=0),[], colormap(ColorMatsp(Xsp~=0),:));
% xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
% ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
% zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
% title(['Time[s]: ',num2str(Times(i,1))]);
% view(180,0)
% ax = gca;
% ax.ZDir = 'reverse'

%時空間分解能計算
% sum(Pointcnt(10:end))/(size(Pointcnt,1)-10);
% sum(Ranges(10:end))/(size(Ranges,1)-10);
% sum(difftime(10:end))/(size(difftime,1)-10);
