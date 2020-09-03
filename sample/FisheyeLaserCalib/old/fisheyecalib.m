%% Videoファイルから画像の読み込み
vidObj = VideoReader('202009012005_video.mp4');
s = struct('cdata',zeros(vidObj.Height,vidObj.Width,3,'uint8'),'colormap',[]);
id = 1;
allFrame = read(vidObj); %すべてのFrameを読み取る
%%
step = 10;
calibimg = allFrame(:,:,:,1:step:size(allFrame,4));
%%
[imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
%%
squareSize = 32;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%%
imageSize = [size(allFrame, 1), size(allFrame, 2)];
fisheyeParams = estimateFisheyeParameters(imagePoints, worldPoints, imageSize);
%%
J = undistortFisheyeImage(allFrame(:,:,:,100),fisheyeParams.Intrinsics);
imshow(J);
%%
imgPoint = detectCheckerboardPoints(allFrame(:,:,:,100));
[R,t] = extrinsics(imgPoint,worldPoints,fisheyeParams.Intrinsics);
newWorldPoints = pointsToWorld(fisheyeParams.Intrinsics,R,t,imgPoint);
figure
plot(worldPoints(:,1),worldPoints(:,2),'gx');
hold on
plot(newWorldPoints(:,1),newWorldPoints(:,2),'ro');
legend('Ground Truth','Estimates');
hold off
%% CSVで出力
%計算結果の保存
csvfile ='param.csv';
fid = fopen(csvfile,'w');
%Cameraパラメータ１
fprintf(fid,'%.15f,',fisheyeParams.Intrinsics.MappingCoefficients);
fprintf(fid,'\n');
fprintf(fid,'%f,',fisheyeParams.Intrinsics.StretchMatrix);
fprintf(fid,'\n');
fprintf(fid,'%f,',fisheyeParams.Intrinsics.DistortionCenter);
fprintf(fid,'\n');
fprintf(fid,'%f,',fisheyeParams.RotationMatrices(:,:,1));
fprintf(fid,'\n');
fclose(fid);