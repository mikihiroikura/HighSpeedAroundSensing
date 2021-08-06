function [opt_planeparams] = planefitting_func(Points, randnorms, maxloopcnt)
% 平面フィッティングの関数
    arguments
        Points = []
        randnorms = 0.001
        maxloopcnt = 3
    end

    %平面最適化で式を計算
    func = @(param)calcplane_func(param, Points);
    min_fval = 1e+20;
    for cnt = 1:maxloopcnt
        x0 = -rand(1,3)*randnorms;%0.001
        options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
        [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
        if exitflag==1&&fval<3000
            opt_planeparams = planeparams;
            break;
        elseif exitflag==1
            if min_fval > fval
                min_fval = fval;
                opt_planeparams = planeparams;
            end
        end
    end

    %外れている輝点を取り除く 閾値10
    dist = abs(1-(opt_planeparams(1).*Points(:,1)+opt_planeparams(2) ...
            .*Points(:,2)+opt_planeparams(3).*Points(:,3))) ...
            ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    Opt_Points = Points(dist<10,:);
    %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
    func = @(param)calcplane_func(param, Opt_Points);
    opt_min_fval = 1e+20;
    for cnt = 1:maxloopcnt
        x0 = -rand(1,3)*randnorms;%0.001
        options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
        [planeparams,fval,exitflag,output] = fminsearch(func,x0,options);
        if exitflag==1&&fval<3000
            opt_planeparams = planeparams;
            break;
        elseif exitflag==1
            if opt_min_fval > fval
                opt_min_fval = fval;
                opt_planeparams = planeparams;
            end
        end
    end
end

