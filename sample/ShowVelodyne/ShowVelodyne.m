Filename = 'data\2021-07-09-16-19-44_Velodyne-HDL-32-Data.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
frameno = 10;
pcobj = readFrame(velodyne, frameno);
figure
pcshow(pcobj);