function []=plot_dist_angle_err(dist_err_trace,angle_err_trace) 

figure(88)
 subplot(1,2,1);plot(dist_err_trace(:,2))
 title('Distance error (mm)')
 subplot(1,2,2);plot(angle_err_trace(:,2))
 title('Angle error (degree)')