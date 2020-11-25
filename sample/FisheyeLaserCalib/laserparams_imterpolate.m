function laserparams_imterpolate()
%laserparams_imterpolate レーザ平面の式と参照面の輝点座標の関連付け
    
    %平面Calibrationのパラメータを呼び出す
    load linelaserparams.mat All_planeparams All_refPoints ref_center ref_radi
    load setup_rgb.mat interpolate_outputfile ref_arcwidth
    
    %All_planeparamsをそれぞれの成分ごとに補完する
    xr = All_refPoints(:,1)-1;
    yr = All_refPoints(:,2)-1;
    sfs = {};
    for i=1:size(All_planeparams,2)
        sf = fit([xr,yr],All_planeparams(:,i),'poly33','Normalize','off');
        sfs{i} = sf;
    end
    
    %三次元点の内挿
%     Fs = {};
%     for i=1:size(All_planeparams,2)
%         F = scatteredInterpolant(xr,yr,All_planeparams(:,i));
%         Fs{i} =F;
%     end
    
    %Camera parameterのCSVへの保存(C++のプログラム用)
    fid = fopen(interpolate_outputfile,'w');
    for i = 1:size(sfs,2)
        fprintf(fid,'%.15f,',coeffvalues(sfs{i}));
        fprintf(fid,'\n');
    end
    fprintf(fid,'%f,',ref_center-1);
    fprintf(fid,'\n');
    fprintf(fid,'%f,',ref_radi);
    fprintf(fid,'\n');
    fprintf(fid,'%f,',ref_arcwidth);
    fprintf(fid,'\n');
    fclose(fid);
end

%デバッグ
%plot(sfs{1},[xr,yr], All_planeparams(:,1));
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
