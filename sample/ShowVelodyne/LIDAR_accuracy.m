% LIDARが取得した点群から，CB平面上にある点群の計測誤差の計算


%パラメータ設定
lidardir = 'data\calibration\LIDAR\';

%LIDARの点群取得
i=2;
lidarname = strcat(lidardir, strcat(num2str(i), '.pcap'));
velodyne = velodyneFileReader(lidarname,'HDL32E');
pcobj = readFrame(velodyne, int8(velodyne.NumberOfFrames/2));
X = pcobj.Location(:,:,1);
Y = pcobj.Location(:,:,2);
Z = pcobj.Location(:,:,3);
cb_area = [-0.8 -0.6;
        -0.08 0.1;
        -0.1 0.15];%CBの範囲指定
id = pcobj.Location(:,:,1) > cb_area(1,1) & pcobj.Location(:,:,1) < cb_area(1,2) & ...
pcobj.Location(:,:,2) > cb_area(2,1) & pcobj.Location(:,:,2) < cb_area(2,2) & ...
pcobj.Location(:,:,3) > cb_area(3,1) & pcobj.Location(:,:,3) < cb_area(3,2);


Checker_cameraPoints = double([X(id), Y(id),Z(id)]);
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

%平面との距離[mm]
dists = (opt_planeparams(1).*Checker_cameraPoints(:,1) + opt_planeparams(2).*Checker_cameraPoints(:,2) ...
    + opt_planeparams(3).*Checker_cameraPoints(:,3) - 1)./norm(opt_planeparams)*1000;
mean(dists)
std(dists)
