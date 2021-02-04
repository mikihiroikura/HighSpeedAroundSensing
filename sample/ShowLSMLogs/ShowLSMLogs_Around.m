%CSVの読み取り
M =csvread('csvs/210201212150_LSM_result.csv');
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
ref_center = [938.469081,492.666857];
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

%点群の距離ごとに色を指定
Ac = 1500;
Dc = 1000;
dist = (Xs.^2 + Ys.^2 + Zs.^2).^0.5;
ColorMat = int8(ones(size(dist)));
ColorMat = ColorMat + int8(dist > Ac);
ColorMat = ColorMat + int8(dist > Dc);
colormap = [1 0 0
            0 1 0
            0 0 1];
        
%参照面の輝点が参照面中央に対してどの方向にあるか
dirs = refpts - ref_center;
rads = atan2(dirs(:,2),dirs(:,1));


%動画と画像の保存先指定
format = 'yyyymmddHHMM';
imgfolder = strcat('D:/Github_output/HighSpeedAroundSensing/ShowLSMLogs/',datestr(now,format));
mkdir(imgfolder);
v = VideoWriter(strcat(imgfolder,'/LSM_result'),'MPEG-4');
open(v)
dirid = 1;
hold off
for i=1:size(Times,1)
    scatter3(Xs(i,(Xs(i,:)~=0)),Ys(i,(Xs(i,:)~=0)),Zs(i,(Xs(i,:)~=0)),[], colormap(ColorMat(i,(Xs(i,:)~=0),:)));
    xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
    ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
    zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
%     xlim([-2000 1000]);
%     ylim([-1500 1000]);
%     zlim([-0 2000]);
    daspect([1 1 1]);
    hold on
    %ここから計測モードによってどれだけHold onするか決める
    if LSM_rotmode(i) == 1.0%局所領域計測時
        if LSM_rotdir(i) ~= LSM_rotdir(i-1)%回転方向が変化したとき
            title(['Time[s]: ',num2str(Times(i,1))]);
            daspect([1 1 1]);
            frame =getframe(gcf);
            writeVideo(v,frame);
            imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid),'.png')));
            saveas(gcf,imgfile);
            dirid = dirid +1;
            hold off
        end
    else%全周計測時
        if LSM_rotdir(i) == 0%右回転
            if rads(i) > 0 && rads(i-1) < 0 %方向ベクトルが+X軸を超えた時
                title(['Time[s]: ',num2str(Times(i,1))]);
                daspect([1 1 1]);
                frame =getframe(gcf);
                writeVideo(v,frame);
                imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid),'.png')));
                saveas(gcf,imgfile);
                dirid = dirid +1;
                hold off
            end
        else%左回転
            if rads(i) < 0 && rads(i-1) > 0 %方向ベクトルが+X軸を超えた時
                title(['Time[s]: ',num2str(Times(i,1))]);
                daspect([1 1 1]);
                frame =getframe(gcf);
                writeVideo(v,frame);
                imgfile = strcat(imgfolder,strcat('/frame_',strcat(sprintf("%03d",dirid),'.png')));
                saveas(gcf,imgfile);
                dirid = dirid +1;
                hold off
            end
        end
    end

end
close(v);

%デバッグ
%空間分解能の計算
% deg = 10;
% ar = 2;
% Xc = Xs(dirchangeid(ar):dirchangeid(ar+1),:);
% Yc = Ys(dirchangeid(ar):dirchangeid(ar+1),:);
% Zc = Zs(dirchangeid(ar):dirchangeid(ar+1),:);
% normar = (Xc.^2+Yc.^2).^0.5;
% cosar = -Yc./normar;
% numar = sum(sum(cosar>cos(deg2rad(deg))));
% lo = 77;
% Xl = Xs(dirchangeid(lo):dirchangeid(lo+1),:);
% Yl = Ys(dirchangeid(lo):dirchangeid(lo+1),:);
% Zl = Zs(dirchangeid(lo):dirchangeid(lo+1),:);
% normlo = (Xl.^2+Yl.^2).^0.5;
% coslo = -Yl./normlo;
% numlo = sum(sum(coslo>cos(deg2rad(deg))));
% Tdif = Times(dirchangeid);
% Tscan = Tdif(2:end)-Tdif(1:end-1);
% hz_ar = 1/(sum(Tscan(6:14))/9);
% hz_lo = 1/(sum(Tscan(16:65))/50);