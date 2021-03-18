function refplane_calibration()
    load setup_rgb.mat refplane_dir ...
        ref_squareSize ref_diffs ref_norm ref_maxcnt ref_step
    load fishparams.mat fisheyeParams

    Refs_cameraPoints = [];
    vidObj = VideoReader(refplane_dir);
    allFrame = read(vidObj); %すべてのFrameを読み取る
    calibimg = allFrame(:,:,:,1:ref_step:size(allFrame,4));
    [imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
    worldPoints = generateCheckerboardPoints(boardSize, ref_squareSize);
    %チェッカーボード(World)toカメラの外部パラメータの計算
    for i=1:size(imagePoints,3)
        [R,t] = extrinsics(imagePoints(:,:,i),worldPoints,fisheyeParams.Intrinsics);
        newrefworldPoints = [worldPoints, zeros(size(imagePoints,1),1)] + ref_diffs;
        cameraPoints = newrefworldPoints * R + t;
        Refs_cameraPoints = [Refs_cameraPoints;cameraPoints];
    end
    %平面最適化で式を計算
    func = @(param)calcplane_func(param, Refs_cameraPoints);
    min_fval = 1e+20;
    for cnt = 1:ref_maxcnt
        x0 = -rand(1,3)*ref_norm;%0.001
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
    dist = abs(1-(opt_planeparams(1).*Refs_cameraPoints(:,1)+opt_planeparams(2) ...
            .*Refs_cameraPoints(:,2)+opt_planeparams(3).*Refs_cameraPoints(:,3))) ...
            ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
    Opt_CheckerRef_cameraPoints = Refs_cameraPoints(dist<10,:);
    %閾値以下のレーザ輝点群を最小二乗法で一つの平面を再出力
    func = @(param)calcplane_func(param, Opt_CheckerRef_cameraPoints);
    opt_min_fval = 1e+20;
    for cnt = 1:ref_maxcnt
        x0 = -rand(1,3)*ref_norm;%0.001
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
    
    refplaneparams = opt_planeparams;

    save refplane_params.mat refplaneparams
end


% f = figure;
% scatter3(Refs_cameraPoints(:,1),Refs_cameraPoints(:,2),Refs_cameraPoints(:,3));
% hold on
% graphX = linspace(min(Refs_cameraPoints(:,1)),max(Refs_cameraPoints(:,1)),50);
% graphY = linspace(min(Refs_cameraPoints(:,2)),max(Refs_cameraPoints(:,2)),50);
% [gX,gY] = meshgrid(graphX,graphY);
% gZ = -opt_planeparams(1)/opt_planeparams(3)*gX-opt_planeparams(2)/opt_planeparams(3)*gY+1/opt_planeparams(3);
% s = mesh(gX,gY,gZ);
% daspect([1 1 1]);