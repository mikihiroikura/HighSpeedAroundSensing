M =csvread('LSM_result.csv');
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
v = VideoWriter('LSM_result','MPEG-4');
open(v)
for i=1:size(Times,1)
    scatter3(Xs(i,(Xs(i,:)~=0)),Ys(i,(Xs(i,:)~=0)),Zs(i,(Xs(i,:)~=0)));
    xlim([min(min(Xs(Xs~=0))) max(max(Xs(Xs~=0)))]);
    ylim([min(min(Ys(Ys~=0))) max(max(Ys(Ys~=0)))]);
    zlim([min(min(Zs(Zs~=0))) max(max(Zs(Zs~=0)))]);
    title(['Time[s]: ',num2str(Times(i,1))]);
    frame =getframe(gcf);
    writeVideo(v,frame);
end
close(v);