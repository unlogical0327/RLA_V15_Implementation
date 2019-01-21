% This program plot velocity and acceleration with time
function []=plot_vel_acc_time(vel_trace,acc_trace)

 figure(120)
 subplot(1,2,1);plot(vel_trace(:,1),'-r');hold on; plot(vel_trace(:,2),'-b')
 title('x and y velocity')
 subplot(1,2,2);plot(acc_trace(:,1),'-r');hold on;plot(acc_trace(:,2),'-b')
 title('x and y acceleration') 