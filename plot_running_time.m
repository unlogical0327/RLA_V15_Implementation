% This program plot velocity and acceleration with time
function []=plot_running_time(tlapsed_m,rmse_trace)

 figure(110)
 subplot(1,2,1);plot(tlapsed_m)
 title('measurement mode loop time')
 subplot(1,2,2);plot(rmse_trace)
 title('measrement rmse error')
