%% 各種パラメータ
csvname = 'csvs/rpm110_axis0_rev5.csv';%保存した点群のCSV
videoname = 'videos/axis0.mp4';%チェッカーボード検出のための動画
squareSize = 32;
load fishparams.mat fisheyeParams;
maxcnt = 3;
fish_step = 2;
evalnums = 10000;%評価に使う点数の合計
norm = -0.001;


%% チェッカーボードの検出
Checker_cameraPoints = [];
vidObj = VideoReader(videoname);
allFrame = read(vidObj); %すべてのFrameを読み取る
calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
[imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%チェッカーボード(World)toカメラの外部パラメータの計算
for i=1:size(imagePoints,3)
    [R,t] = extrinsics(imagePoints(:,:,i),worldPoints,fisheyeParams.Intrinsics);
    newworldPoints = [worldPoints, zeros(size(imagePoints,1),1)];
    cameraPoints = newworldPoints * R + t;
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

%% チェッカーボードの出力
f = figure;
scatter3(Checker_cameraPoints(:,1),Checker_cameraPoints(:,2),Checker_cameraPoints(:,3));
hold on
graphX = linspace(min(Checker_cameraPoints(:,1)),max(Checker_cameraPoints(:,1)),50);
graphY = linspace(min(Checker_cameraPoints(:,2)),max(Checker_cameraPoints(:,2)),50);
[gX,gY] = meshgrid(graphX,graphY);
gZ = -opt_planeparams(1)/opt_planeparams(3)*gX-opt_planeparams(2)/opt_planeparams(3)*gY+1/opt_planeparams(3);
s = mesh(gX,gY,gZ);
daspect([1 1 1]);

%% チェッカーボード範囲指定
checkXmins = [-200, -200, -200];
checkXmaxs = [100, 100, 100];
checkZmins = [280, 300, 350];
checkZmaxs = [480, 500, 550];
checkid = 3;
checkXmin = checkXmins(checkid);
checkXmax = checkXmaxs(checkid);
checkZmin = checkZmins(checkid);
checkZmax = checkZmaxs(checkid);
%% CSVの読み取り
M =csvread(csvname);
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
refpts = M(1:4:end,12:13);
onchecker_id = (Xs~=0) & (Xs > checkXmin) & (Xs < checkXmax) & (Zs > checkZmin) & (Zs < checkZmax);
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

%% 評価
dist_eval = (1-(opt_planeparams(1).*X_eval(:)+opt_planeparams(2) ...
        .*Y_eval(:)+opt_planeparams(3).*Z_eval(:))) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
dist_fromcam = (1-(opt_planeparams(1)*0+opt_planeparams(2) ...
        *0+opt_planeparams(3)*0)) ...
        ./(opt_planeparams(1)^2+opt_planeparams(2)^2+opt_planeparams(3)^2)^0.5;
%% 評価のグラフ表示(計測点群とチェッカーボード平面との距離の相関)
dist_eval_nooutlier = dist_eval > -5000 & dist_eval < 5000;
figure;
colormap jet;
scatter3(X_eval(dist_eval_nooutlier), Y_eval(dist_eval_nooutlier), Z_eval(dist_eval_nooutlier), ...
[], dist_eval(dist_eval_nooutlier));
daspect([1 1 1]);
%% 評価のグラフ(参照面の輝点と計測誤差との相関関係)
figure;
colormap jet;
scatter(Refs_eval(:,1), Refs_eval(:,2),[], dist_eval);
daspect([1 1 1]);
%% 評価結果
mean(dist_eval)
std(dist_eval)