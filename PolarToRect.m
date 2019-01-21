% this program change polar coordinate data to rectangular x-y coordinate
function [calibration_data,scan_data]=PolarToRect(Lidar_data)
% Lidar data --1 scan angle; --2 distance; --3 amplitude
calibration_data=0;
[l,w]=size(Lidar_data);
    for ii=1:w
        calibration_data(ii,1) = cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        calibration_data(ii,2) = sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        if l>=3
        calibration_data(ii,3) = Lidar_data(3,ii);
        end
    end
    scan_data = Lidar_data;
    %plot(calibration_data(:,1),calibration_data(:,2),'.');


