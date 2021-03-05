%% 各種パラメータ
csvname = 'csvs/210217132019_LSM_result_demo_iros2021.csv';%保存した点群のCSV
videoname = 'videos/--.mp4';%チェッカーボード検出のための動画
squareSize = 32;
load fishparams.mat fisheyeParams;
maxcnt = 3;
evalnums = 10000;%評価に使う点数の合計


%% チェッカーボードの検出
Checker_cameraPoints = [];
vidObj = VideoReader(videoname);
allFrame = read(vidObj); %すべてのFrameを読み取る
calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
[imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%チェッカーボード(World)toカメラの外部パラメータの計算
RotMatrix = zeros(3,3,size(imagePoints,3));
TransVec = zeros(size(imagePoints,3), 3);
for i=1:size(imagePoints,3)
    [R,t] = extrinsics(imagePoints(:,:,i),worldPoints,fisheyeParams.Intrinsics);
    newworldPoints = [imagePoints(:,:,i), zeros(size(imagePoints,1),1)];
    cameraPoints = newworldPoints * R + T;
    Checker_cameraPoints = [Checker_cameraPoints;cameraPoints];
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


%% CSVの読み取り
M =csvread(csvname);
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
X = reshape(Xs(Xs~=0), [nnz(Xs), 1]);
Y = reshape(Ys(Ys~=0), [nnz(Ys), 1]);
Z = reshape(Zs(Zs~=0), [nnz(Zs), 1]);
idx = randperm(size(X,1), evalnums);
X_eval = X(idx);
Y_eval = Y(idx);
Z_eval = Z(idx);

%% 評価
dist_eval = abs(1-(opt_planeparams(1).*X_eval(:)+opt_planeparams(2) ...
        .*Y_eval(:)+opt_planeparams(3).*Z_eval(:))) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;