Filename = 'data\evaluation\LIDAR\2021-10-07-16-27-23_Velodyne-HDL-32-Data.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
frameno = 18;
pcobj = readFrame(velodyne, frameno);
figure
pcshow(pcobj);