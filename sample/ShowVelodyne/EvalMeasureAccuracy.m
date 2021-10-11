%%% MDPI Sensorsで点群の時空間分解能，平面フィッティングで
%%% 計測精度の評価
%% 各種パラメータ
csvname = 'data\evaluation\Camera\210803203212_LSM_result_rotate_thesiscand.csv';%保存した点群のCSV
imgname = 'data\evaluation\Checkerboard\202108031923_img00.png';%チェッカーボード検出のための動画
squareSize = 28.3;
fish_step = 2;
evalnums = 10000;%評価に使う点数の合計
%カメラパラメータ読み取り
camparamdir = '202103151940_fisheyeparam.csv';
Mfish = csvread(camparamdir);
mapcoeff = Mfish(1,1:4);
imagesize = [896 896];
distortion = Mfish(3,1:2)+1;
intrinsics = fisheyeIntrinsics(mapcoeff,imagesize,distortion);


%% チェッカーボードの検出
[imagePoints, boardSize] = detectCheckerboardPoints(imgname);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[R, t] = extrinsics(imagePoints, worldPoints, intrinsics);
norm_cam2cb = R(3,:) * dot(R(3,:),t);

%CBの点群をCamera座標系に変換
worldPts = [worldPoints,zeros(size(worldPoints,1),1)];
Checker_cameraPoints = worldPts * R + t;


maxcnt = 3;
norms = -0.001;
%平面最適化で式を計算
func = @(param)calcplane_func(param, Checker_cameraPoints);
min_fval = 1e+20;
for cnt = 1:maxcnt
    x0 = -rand(1,3)*norms;%0.001
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
    x0 = -rand(1,3)*norms;%0.001
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
checkXmins = [-50, -10000];
checkXmaxs = [50, -700];
checkYmins = [-10000,-220];
checkYmaxs = [10000, 30];
checkZmins = [380, 280];
checkZmaxs = [450, 480];
checkid = 1;
checkXmin = checkXmins(checkid);
checkXmax = checkXmaxs(checkid);
checkYmin = checkYmins(checkid);
checkYmax = checkYmaxs(checkid);
checkZmin = checkZmins(checkid);
checkZmax = checkZmaxs(checkid);
%% CSVの読み取り
M =csvread(csvname);
Times = M(1:4:end,1);
Xs = M(2:4:end,:);
Ys = M(3:4:end,:);
Zs = M(4:4:end,:);
refpts = M(1:4:end,12:13);
onchecker_id = (Xs~=0) & (Xs > checkXmin) & (Xs < checkXmax) & (Zs > checkZmin) & (Zs < checkZmax) & (Ys > checkYmin) & (Ys < checkYmax);
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
%% 計測精度評価結果
mean(dist_eval)
std(dist_eval)

%% 計測点群の平面フィッティング
PointEvals = [X_eval, Y_eval, Z_eval];
planeparams_measured = planefitting_func(PointEvals);
%% 平面フィッティング結果による計測精度評価
dist_eval_measured = (1-(planeparams_measured(1).*X_eval(:)+planeparams_measured(2) ...
        .*Y_eval(:)+planeparams_measured(3).*Z_eval(:))) ...
        ./(planeparams_measured(1)^2+planeparams_measured(2)^2+planeparams_measured(3)^2)^0.5;
mean(dist_eval_measured)
std(dist_eval_measured)
%% 平面フィッティング結果の出力
f = figure;
scatter3(PointEvals(:,1),PointEvals(:,2),PointEvals(:,3));
hold on
graphX = linspace(min(PointEvals(:,1)),max(PointEvals(:,1)),50);
graphY = linspace(min(PointEvals(:,2)),max(PointEvals(:,2)),50);
[gX,gY] = meshgrid(graphX,graphY);
gZ = -planeparams_measured(1)/planeparams_measured(3)*gX-planeparams_measured(2)/planeparams_measured(3)*gY+1/planeparams_measured(3);
s = mesh(gX,gY,gZ);
xlim([checkXmin checkXmax]);
zlim([checkZmin checkZmax]);
daspect([1 1 1]);
%% 時空間分解能と計測範囲評価
% MDPI用の図はこれではない
ref_center = [445.068326,404.6309];
LSM_rotdir = M(1:4:end,2);
LSM_detectedenablefig = M(1:4:end,3);
LSM_objdetectedflg = M(1:4:end,4);
LSM_reciprocntdown = M(1:4:end,5);
LSM_alertcnt = M(1:4:end,6);
LSM_dangercnt = M(1:4:end,7);
LSM_rpms = M(1:4:end,8);
LSM_laserplane_nml = M(1:4:end,9:11);
refpts = M(1:4:end,12:13);
LSM_rotmode = M(1:4:end,14);
%参照面の輝点が参照面中央に対してどの方向にあるか
dirs = refpts - ref_center;
rads = atan2(dirs(:,2),dirs(:,1));
Modes = [];
difftime = [];
Pointcnt = [];
Pointcntoncb = [];
Ranges = [];
Timeold = 0;
drads = 0;
%出力図の相フレーム数の指定
%0ならば自動で出力の時間を変える
%それ以外なら指定の出力フレーム数
framenum = 0;

idold = 1;
for i=2:size(Times,1)
    if abs(rads(i)-rads(i-1)) > 5
        drads =drads - abs(rads(i)-rads(i-1)) + 2 * pi;
    else
        drads =drads + abs(rads(i)-rads(i-1));
    end
    %ここから計測モードによってどれだけHold onするか決める
    if framenum == 0
        if LSM_rotmode(i) == 1.0%局所領域計測時
            if LSM_rotdir(i) ~= LSM_rotdir(i-1)%回転方向が変化したとき
                Xsp = Xs(idold:i,:);
                Ysp = Ys(idold:i,:);
                Zsp = Zs(idold:i,:);
                onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                idold = i;
                %時空間分解能計算
                Modes = [Modes;"ReciprLR"];
                Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                difftime =[difftime;Times(i)-Timeold];
                Ranges = [Ranges; drads];
                drads = 0;
                Timeold = Times(i);
            end
        else%全周計測時
            if LSM_rotdir(i) == 0%右回転
                if rads(i) > 0 && rads(i-1) < 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                    Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                    Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                    Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-R"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                end
            else%左回転
                if rads(i) < 0 && rads(i-1) > 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                    Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                    Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                    Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-L"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                end
            end
        end
    else
        if mod(i,framenum) == 0
            Xsp = Xs(idold:i,:);
            Ysp = Ys(idold:i,:);
            Zsp = Zs(idold:i,:);
            idold = i;
        end
    end
end

%% 時空間分解能計測範囲計算
setid = Ranges > 6;
measuredarea = sum(Ranges(setid))/sum(setid)*180/pi
spatialresol = sum(Pointcntoncb(setid))/sum(setid)/((checkXmax-checkXmin)* (checkZmax-checkZmin)/100)
tempresol = 1/(sum(difftime(setid))/sum(setid))

%% MDPI用の図の作成
% 再度モードごとに点群を保存

%出力点群の範囲再度指定
checkXmin = -70;
checkXmax = 70;
checkZmin = 360;
checkZmax = 560;

ref_center = [445.068326,404.6309];
LSM_rotdir = M(1:4:end,2);
LSM_detectedenablefig = M(1:4:end,3);
LSM_objdetectedflg = M(1:4:end,4);
LSM_reciprocntdown = M(1:4:end,5);
LSM_alertcnt = M(1:4:end,6);
LSM_dangercnt = M(1:4:end,7);
LSM_rpms = M(1:4:end,8);
LSM_laserplane_nml = M(1:4:end,9:11);
refpts = M(1:4:end,12:13);
LSM_rotmode = M(1:4:end,14);
%参照面の輝点が参照面中央に対してどの方向にあるか
dirs = refpts - ref_center;
rads = atan2(dirs(:,2),dirs(:,1));
Modes = [];
difftime = [];
Pointcnt = [];
Pointcntoncb = [];
Ranges = [];
Timeold = 0;
drads = 0;
%出力図の相フレーム数の指定
%0ならば自動で出力の時間を変える
%それ以外なら指定の出力フレーム数
framenum = 0;
%MDPI用の図のための変数
Xoncbs = {};
Yoncbs = {};
Zoncbs = {};
distoncbs = {};
cellid = 1;
% 平面の法線ベクトルがX軸になるような平面に変換する
planenorm = planeparams_measured/norm(planeparams_measured);
inplane_z0 = [planenorm(2) -planenorm(1) 0]/norm(planenorm(1:2));
inplane_other = cross(planenorm,inplane_z0);
R_orig2view = inv([planenorm;inplane_z0;inplane_other]);

idold = 1;
for i=2:size(Times,1)
    if abs(rads(i)-rads(i-1)) > 5
        drads =drads - abs(rads(i)-rads(i-1)) + 2 * pi;
    else
        drads =drads + abs(rads(i)-rads(i-1));
    end
    %ここから計測モードによってどれだけHold onするか決める
    if framenum == 0
        if LSM_rotmode(i) == 1.0%局所領域計測時
            if LSM_rotdir(i) ~= LSM_rotdir(i-1)%回転方向が変化したとき
                Xsp = Xs(idold:i,:);
                Ysp = Ys(idold:i,:);
                Zsp = Zs(idold:i,:);
                onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                idold = i;
                %時空間分解能計算
                Modes = [Modes;"ReciprLR"];
                Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                difftime =[difftime;Times(i)-Timeold];
                Ranges = [Ranges; drads];
                drads = 0;
                Timeold = Times(i);
                %回転
                Ptsoncb = [Xoncb, Yoncb, Zoncb] * R_orig2view;
                transplanenorm = planeparams_measured * R_orig2view;
                %点群をScanごとに保存
                if size(Ranges,1)~=1
                    Xoncbs{cellid} = Ptsoncb(:,1);
                    Yoncbs{cellid} = Ptsoncb(:,2);
                    Zoncbs{cellid} = Ptsoncb(:,3);
                    dist_oncb = (1-(transplanenorm(1).*Ptsoncb(:,1)+transplanenorm(2) ...
                                .*Ptsoncb(:,2)+transplanenorm(3).*Ptsoncb(:,3))) ...
                                ./(transplanenorm(1)^2+transplanenorm(2)^2+transplanenorm(3)^2)^0.5;
                    distoncbs{cellid} = dist_oncb;
                    cellid = cellid + 1;
                end
            end
        else%全周計測時
            if LSM_rotdir(i) == 0%右回転
                if rads(i) > 0 && rads(i-1) < 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                    Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                    Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                    Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-R"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                    %回転
                    Ptsoncb = [Xoncb, Yoncb, Zoncb] * R_orig2view;
                    transplanenorm = planeparams_measured * R_orig2view;
                    %点群をScanごとに保存
                    if size(Ranges,1)~=1
                        Xoncbs{cellid} = Ptsoncb(:,1);
                        Yoncbs{cellid} = Ptsoncb(:,2);
                        Zoncbs{cellid} = Ptsoncb(:,3);
                        dist_oncb = (1-(transplanenorm(1).*Ptsoncb(:,1)+transplanenorm(2) ...
                                    .*Ptsoncb(:,2)+transplanenorm(3).*Ptsoncb(:,3))) ...
                                    ./(transplanenorm(1)^2+transplanenorm(2)^2+transplanenorm(3)^2)^0.5;
                        distoncbs{cellid} = dist_oncb;
                        cellid = cellid + 1;
                    end
                end
            else%左回転
                if rads(i) < 0 && rads(i-1) > 0 %方向ベクトルが+X軸を超えた時
                    Xsp = Xs(idold:i,:);
                    Ysp = Ys(idold:i,:);
                    Zsp = Zs(idold:i,:);
                    onchecker_id = (Xsp~=0) & (Xsp > checkXmin) & (Xsp < checkXmax) & (Zsp > checkZmin) & (Zsp < checkZmax) & (Ysp > checkYmin) & (Ysp < checkYmax);
                    Xoncb = reshape(Xsp(onchecker_id), [size(Xsp(onchecker_id),1), 1]);
                    Yoncb = reshape(Ysp(onchecker_id), [size(Ysp(onchecker_id),1), 1]);
                    Zoncb = reshape(Zsp(onchecker_id), [size(Zsp(onchecker_id),1), 1]);
                    idold = i;
                    %時空間分解能計算
                    Modes = [Modes;"Rotate-L"];
                    Pointcnt = [Pointcnt;sum(sum(Xsp~=0))];
                    Pointcntoncb = [Pointcntoncb;sum(sum(Xoncb~=0))];
                    difftime =[difftime;Times(i)-Timeold];
                    Ranges = [Ranges; drads];
                    drads = 0;
                    Timeold = Times(i);
                    %回転
                    Ptsoncb = [Xoncb, Yoncb, Zoncb] * R_orig2view;
                    transplanenorm = planeparams_measured * R_orig2view;
                    %点群をScanごとに保存
                    if size(Ranges,1)~=1
                        Xoncbs{cellid} = Ptsoncb(:,1);
                        Yoncbs{cellid} = Ptsoncb(:,2);
                        Zoncbs{cellid} = Ptsoncb(:,3);
                        dist_oncb = (1-(transplanenorm(1).*Ptsoncb(:,1)+transplanenorm(2) ...
                                    .*Ptsoncb(:,2)+transplanenorm(3).*Ptsoncb(:,3))) ...
                                    ./(transplanenorm(1)^2+transplanenorm(2)^2+transplanenorm(3)^2)^0.5;
                        distoncbs{cellid} = dist_oncb;
                        cellid = cellid + 1;
                    end
                end
            end
        end
    else
        if mod(i,framenum) == 0
            Xsp = Xs(idold:i,:);
            Ysp = Ys(idold:i,:);
            Zsp = Zs(idold:i,:);
            idold = i;
        end
    end
end

% 同じScan回数分の点群を出力する
scannum = 20;
scanid = randperm(size(Xoncbs,2),scannum);
f = figure;
colormap jet;
Clim = [-30 30];
for i=scanid
    scatter3(Xoncbs{i}, Yoncbs{i}, Zoncbs{i}, ...
    [], distoncbs{i},'o','filled');
    hold on
%     if Clim(1)>min(distoncbs{i})
%         Clim(1) = min(distoncbs{i});
%     end
%     if Clim(2)<max(distoncbs{i})
%         Clim(2) = max(distoncbs{i});
%     end
end
daspect([1 1 1]);
graphZ = linspace(-850,-250,13);
graphY = linspace(-400,200,13);
[gZ,gY] = meshgrid(graphZ,graphY);
gX = -transplanenorm(2)/transplanenorm(1)*gY-transplanenorm(3)/transplanenorm(1)*gZ+1/transplanenorm(1);
s = mesh(gX,gY,gZ,'EdgeColor',[0 0 0], 'FaceColor', 'none', 'FaceAlpha', 1);
xlabel('X [mm]','FontSize',12);
ylabel('Y [mm]','FontSize',12);
zlabel('Z [mm]','FontSize',12);
ax = gca;
ax.CLim = Clim;
grid off;
ax.XAxis.Visible = 'off';
ax.YAxis.Visible = 'off';
ax.ZAxis.Visible = 'off';
margin = 20;
ylim([-150-margin 50+margin]);
zlim([-650-margin -400+margin]);
view(-135,20);
%% Colorbarの設置
c = colorbar('southoutside');
ax.CLim = Clim;
c.FontSize = 12;
c.Label.String = '[mm]';
c.Label.Position = [Clim(2)+2.5 -0.25 0];
%% 図をPDFに保存
print(gcf,'-painters','b','-dpdf');