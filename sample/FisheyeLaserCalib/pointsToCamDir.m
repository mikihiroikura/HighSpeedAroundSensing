function camDir = pointsToCamDir(Intrinsics,points)
%UNTITLED3 この関数の概要をここに記述
%   詳細説明をここに記述
us = points(:,1)-Intrinsics.DistortionCenter(1);
vs = points(:,2)-Intrinsics.DistortionCenter(2);
phis = (us.^2 + vs.^2).^0.5;
ws = Intrinsics.MappingCoefficients(1) + Intrinsics.MappingCoefficients(2).*phis(:).^2 ...
+Intrinsics.MappingCoefficients(3).*phis(:).^3 + Intrinsics.MappingCoefficients(4).*phis(:).^4;
camDir = [us,vs,ws];
end

