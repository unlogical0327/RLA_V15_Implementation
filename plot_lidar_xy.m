% Plot lidar x and y
function []=plot_lidar_xy(Lidar_trace,Lidar_expect_trace)
figure(66);
 subplot(2,2,1);plot(Lidar_trace(:,1))
 title('Lidar x location (mm)')
 subplot(2,2,2);plot(Lidar_trace(:,2))
 title('Lidar y location (mm)')
 subplot(2,2,3);plot(Lidar_expect_trace(:,1))
 title('Lidar expect x location (mm)')
 subplot(2,2,4);plot(Lidar_expect_trace(:,2))
 title('Lidar expect y location (mm)')