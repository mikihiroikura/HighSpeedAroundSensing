function laserparams_imterpolate()
%laserparams_imterpolate レーザ平面の式と参照面の輝点座標の関連付け
    
    %平面Calibrationのパラメータを呼び出す
    load linelaserparams.mat All_planeparams All_refPoints
    load setup.mat interpolate_outputfile
    
    %All_planeparamsをそれぞれの成分ごとに補完する
    xr = All_refPoints(:,1);
    yr = All_refPoints(:,2);
    sfs = {};
    for i=1:size(All_planeparams,2)
        sf = fit([xr,yr],All_planeparams(:,i),'poly22');
        sfs{i} = sf;
    end
    
    %Camera parameterのCSVへの保存(C++のプログラム用)
    fid = fopen(interpolate_outputfile,'w');
    for i = 1:size(sfs,2)
        fprintf(fid,'%f,',coeffvalues(sfs{i}));
        fprintf(fid,'\n');
    end
    fclose(fid);
end

%デバッグ
%plot(sfs{1},[xr,yr], All_planeparams(:,1))