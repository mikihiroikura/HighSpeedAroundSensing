%CSVの読み取り
M =csvread('csvs/201013111144_LSM_result.csv');
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
ref_center = [938.469081,492.666857];
refpts = M(1:4:end,2:3);
dirs = refpts-ref_center;
%座標を傾けたいとき
dirs = dirs * [0 1;-1 0];
degs = rad2deg(atan2(dirs(:,2),dirs(:,1)));
modes = M(1:4:end,4);
%CSVから方向転換する場所の検出
[pks,locs,w,p] = findpeaks(degs);
[mpks,mlocs,mw,mp] = findpeaks(-degs);
dcp = locs;
dcm = mlocs;
dcm = dcm(modes(dcm)==1);
dirchangeid = sort([dcp;dcm]);
%動画と画像の保存先指定
format = 'yyyymmddHHMM';
imgfolder = strcat('D:/Github_output/HighSpeedAroundSensing/ShowLSMLogs/',datestr(now,format));
mkdir(imgfolder);
v = VideoWriter(strcat(imgfolder,'/LSM_result'),'MPEG-4');
open(v)
dirid = 1;
hold off
for i=1:size(Times,1)
    scatter3(Xs(i,(Xs(i,:)~=0)),Ys(i,(Xs(i,:)~=0)),Zs(i,(Xs(i,:)~=0)),'bo');
    xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
    ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
    zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
%     xlim([-2000 1000]);
%     ylim([-1500 1000]);
%     zlim([-0 2000]);
    daspect([1 1 1]);
    hold on
    if dirid<=size(dirchangeid,1)
        if dirchangeid(dirid)==i && dirid<=size(dirchangeid,1)
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