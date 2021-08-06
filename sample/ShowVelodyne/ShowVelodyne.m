Filename = 'data\evaluation\LIDAR\2021-08-03-21-02-32_Velodyne-HDL-32-Data_thesiscand.pcap';
velodyne = velodyneFileReader(Filename,'HDL32E');
frameno = 10;
pcobj = readFrame(velodyne, frameno);
figure
pcshow(pcobj);