%チェッカーボード周囲のLIDAR取得点群の空間分解能計算

Filename = 'data\2021-07-09-21-05-47_Velodyne-HDL-32-Data.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
checkerRanges = [-1, 1;
                -1, 1;
                -1, 1];%チェッカーボード板のある範囲を指定する

totalptcnt = 0;
for i = 1:velodyne.NumberOfFrames
   pcobj = readFrame(velodyne, i);
   id = pcobj.Location(:,:,1) > checkerRanges(1,1) & pcobj.Location(:,:,1) < checkerRanges(1,2) ...
       & pcobj.Location(:,:,2) > checkerRanges(2,1) & pcobj.Location(:,:,2) < checkerRanges(2,2) ...
       & pcobj.Location(:,:,3) > checkerRanges(3,1) & pcobj.Location(:,:,3) < checkerRanges(3,2);
   ptcnt = sum(sum(id));
   totalptcnt = totalptcnt + ptcnt;
end
%全フレームでのチェッカーボード板の範囲内の点数の平均
meanptcnt = totalptcnt / velodyne.NumberOfFrames;

%空間分解能の計算
