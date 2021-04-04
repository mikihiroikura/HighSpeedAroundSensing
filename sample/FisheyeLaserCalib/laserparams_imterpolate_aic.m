function laserparams_imterpolate_aic()
%laserparams_imterpolate レーザ平面の式と参照面の輝点座標の関連付け
    
    %平面Calibrationのパラメータを呼び出す
    load linelaserparams.mat All_planeparams All_refPoints ref_center ref_radi
    load setup_rgb.mat interpolate_outputfile ref_arcwidth ref_arcwidth_margin
    
    %All_planeparamsをそれぞれの成分ごとに補完する
    xr = All_refPoints(:,1);
    yr = All_refPoints(:,2);
    sfs = {};
    all_sfs = {};
    polys = ["poly22", "poly33", "poly44", "poly55"];
    kous = [4*3/2, 5*4/2, 6*5/2, 7*6/2];
    aics = zeros(3,4);
    for j = 1:size(polys,2)
        for i=1:size(All_planeparams,2)
            sf = fit([xr,yr],All_planeparams(:,i),polys(j),'Normalize','off');
            sfs{i} = sf;
            resdiff = sum((sf(xr,yr)-All_planeparams(:,i)).^2);
            aics(i,j) = size(xr,1) * (log(2*pi*resdiff/size(xr,1))+1) + 2*(kous(j)+2);
        end
        all_sfs{j} = sfs;
    end
    
end

%デバッグ
% figure
% plot(sfs{1},[xr,yr], All_planeparams(:,1));%%論文用の図はこれ
% xlabel('u [px]','FontSize',12);
% ylabel('v [px]','FontSize',12);
% zlabel('\it \bf n_0','FontSize',12);

%zlim([min(All_planeparams(:,1)) max(All_planeparams(:,1))])

% 補間の確認
% intpoints = [];
% for i=1:22
%     intpoints = [intpoints;mean(All_refPoints(i:i+1,:))];
% end
% for k =1:3
%   figure
%   plot3(xr,yr,All_planeparams(:,k),'bo');
%   xlim([min() 5])
%   hold on
%   zint = Fs{k}(intpoints(:,1),intpoints(:,2));
%   plot3(intpoints(:,1),intpoints(:,2),zint,'ro');
%   plot3(intpoints(:,1),intpoints(:,2),sfs{k}(intpoints(:,1),intpoints(:,2)),'ko');
%   diff{k} = sfs{k}(intpoints(:,1),intpoints(:,2))-zint;
% end
