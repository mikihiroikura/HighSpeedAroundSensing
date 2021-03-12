%各種パラメータ
axes = [700 350 0];
rpms = [10,110,210,310,410];
videofolder = 'videos/';
csvfolder = 'csvs/';
squareSize = 32;
load fishparams.mat fisheyeParams;
maxcnt = 3;%平面最適化を行う最大回数
fish_step = 2;%チェッカーボード検出用の出力フレーム
evalnums = 10000;%評価に使う点数の合計
norm = -0.001;%最適化の際に初期値の決め方
%チェッカーボード範囲指定
checkXmin = [-200, -200, -200];
checkXmax = [100, 100, 100];
checkZmin = [280, 300, 350];
checkZmax = [480, 500, 550];
%保存用配列
all_planeparams = zeros(size(axes,2),3);%平面の式の保存
dist_fromcams = zeros(size(axes,2),1);%カメラ原点から平面までの距離
dist_evals = zeros(evalnums,size(rpms,2),size(axes,2));
dist_evals_means = zeros(size(axes,2),size(rpms,2));
dist_evals_stds = zeros(size(axes,2),size(rpms,2));
X_evals = zeros(evalnums, size(rpms,2),size(axes,2));
Y_evals = zeros(evalnums, size(rpms,2),size(axes,2));
Z_evals = zeros(evalnums, size(rpms,2),size(axes,2));

%全てのパターンで解析
for k = 1:size(axes,2)
    videoname = strcat(strcat(strcat(videofolder, 'axis'),int2str(axes(k))),'.mp4');
    %チェッカーボード検出
    vidObj = VideoReader(videoname);
    allFrame = read(vidObj); %すべてのFrameを読み取る
    calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
    [imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    Checker_cameraPoints = zeros(size(worldPoints,1) * size(imagePoints,3), 3);
    %チェッカーボード(World)toカメラの外部パラメータの計算
    for i=1:size(imagePoints,3)
        [R,t] = extrinsics(imagePoints(:,:,i),worldPoints,fisheyeParams.Intrinsics);
        newworldPoints = [worldPoints, zeros(size(imagePoints,1),1)];
        cameraPoints = newworldPoints * R + t;
        Checker_cameraPoints(size(cameraPoints,1) * (i-1) + 1:...
            size(cameraPoints,1) * (i),:) = cameraPoints;
    end
    %平面最適化で式を計算
    func = @(param)calcplane_func(param, Checker_cameraPoints);
    min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norm;%0.001
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
    dist = abs(1-(opt_planeparams(1).*Checker_cameraPoints(:,1)+opt_planeparams(2) ...
            .*Checker_cameraPoints(:,2)+opt_planeparams(3).*Checker_cameraPoints(:,3))) ...
            ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    Opt_Checker_cameraPoints = Checker_cameraPoints(dist<10,:);
    %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
    func = @(param)calcplane_func(param, Opt_Checker_cameraPoints);
    opt_min_fval = 1e+20;
    for cnt = 1:maxcnt
        x0 = -rand(1,3)*norm;%0.001
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
    dist_fromcam = (1-(opt_planeparams(1)*0+opt_planeparams(2) ...
        *0+opt_planeparams(3)*0)) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    dist_fromcams(k) = dist_fromcam;
    all_planeparams(k,:) = opt_planeparams;
    
    %全てのRPMでの計測結果出力
    for j = 1:size(rpms,2)
        csvname = strcat(strcat(strcat(strcat(strcat(csvfolder, 'rpm'),...
             int2str(rpms(j))),'_axis'),int2str(axes(k))),'.csv');
        M =csvread(csvname);
        Times = M(1:4:end,1);
        Xs = M(2:4:end,:);
        Ys = M(3:4:end,:);
        Zs = M(4:4:end,:);
        refpts = M(1:4:end,12:13);
        onchecker_id = (Xs~=0) & (Xs > checkXmin(k)) & (Xs < checkXmax(k)) & (Zs > checkZmin(k)) & (Zs < checkZmax(k));
        X = reshape(Xs(onchecker_id), [size(Xs(onchecker_id),1), 1]);
        Y = reshape(Ys(onchecker_id), [size(Ys(onchecker_id),1), 1]);
        Z = reshape(Zs(onchecker_id), [size(Zs(onchecker_id),1), 1]);
        Refs = [];
        refids = sum(int8(onchecker_id),2);
        for i=1:size(refids,1)
            Refs = [Refs;repmat(refpts(i,:),refids(i),1)];
        end
        idx = randperm(size(X,1), evalnums);
        X_eval = X(idx);
        Y_eval = Y(idx);
        Z_eval = Z(idx);
        Refs_eval = Refs(idx, :);
        %評価
        dist_eval = (1-(opt_planeparams(1).*X_eval(:)+opt_planeparams(2) ...
        .*Y_eval(:)+opt_planeparams(3).*Z_eval(:))) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;        
        %保存
        X_evals(:,j,k) = X_eval;
        Y_evals(:,j,k) = Y_eval;
        Z_evals(:,j,k) = Z_eval;
        dist_evals(:,j,k) = dist_eval;
        dist_evals_means(k,j) = mean(dist_eval);
        dist_evals_stds(k,j) = std(dist_eval);
    end
end    