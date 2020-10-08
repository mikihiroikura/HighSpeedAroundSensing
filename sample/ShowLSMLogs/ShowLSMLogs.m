%CSVの読み取り
M =csvread('csvs/LSM_result.csv');
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
ref_center = [938.469081,492.666857];
refpts = M(1:4:end,2:3);
dirs = refpts-ref_center;
degs = rad2deg(atan2(dirs(:,2),dirs(:,1)));
modes = M(1:4:end,4);
%CSVから方向転換する場所の検出
[pks,locs,w,p] = findpeaks(degs);
[mpks,mlocs,mw,mp] = findpeaks(-degs);
dcp = locs(w>10);
dcm = mlocs(mw>10);
dcm = dcm(modes(dcm)==1);
dirchangeid = sort([dcp;dcm]);

v = VideoWriter('LSM_result','MPEG-4');
open(v)
dirid = 1;
hold off
for i=1:size(Times,1)
    scatter3(Xs(i,(Xs(i,:)~=0)),Ys(i,(Xs(i,:)~=0)),Zs(i,(Xs(i,:)~=0)),'bo');
%     xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
%     ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
%     zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
    xlim([-3000 3000]);
    ylim([-3000 3000]);
    zlim([-3000 3000]);
    hold on
    if dirid<=size(dirchangeid,1)
        if dirchangeid(dirid)==i && dirid<=size(dirchangeid,1)
            title(['Time[s]: ',num2str(Times(i,1))]);
            frame =getframe(gcf);
            writeVideo(v,frame);
            imgfile = strcat('rei',strcat(num2str(dirid),'_.png'));
            saveas(gcf,imgfile);
            dirid = dirid +1;
            hold off
        end
    end
end
close(v);