function Error = ReprojectCam2LIDAR(x,norms,lidarpts)
    RotMat = eul2rotm(x(1:3));
    RotMat = [0 1 0;
              1 0 0;
              0 0 -1];
    pcam = (lidarpts) * RotMat + x(4:6);
    er = dot(norms, pcam,2)-(norms(:,1).^2 + norms(:,2).^2 + norms(:,3).^2);
    Error = sum(er);
end