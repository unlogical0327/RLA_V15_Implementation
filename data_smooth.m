% this program make data smooth with previous data trace
function [Lidar_update_xy_wm_sm] = data_smooth(Lidar_trace,Lidar_update_xy_wm,smooth_step)
len=length(Lidar_trace);
sum_trace = [0 0];
if len>smooth_step*4
    for i=1:smooth_step
    sum_trace(1,1)=sum_trace(1,1)+Lidar_trace(len-i,1)
    sum_trace(1,2)=sum_trace(1,2)+Lidar_trace(len-i,2)
    end
    Lidar_update_xy_wm_sm(1,1)=(Lidar_update_xy_wm(1,1)+sum_trace(1,1))/(smooth_step+1)
    Lidar_update_xy_wm_sm(1,2)=(Lidar_update_xy_wm(1,2)+sum_trace(1,2))/(smooth_step+1)
else
    Lidar_update_xy_wm_sm=Lidar_update_xy_wm;
end


