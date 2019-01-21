% This program plot velocity and acceleration with time
function []=plot_rot_acc_time(rotation_trace,rot_vel_trace,rot_acc_trace)

 figure(122)
 subplot(1,3,1);plot(rotation_trace)
 title('rotation angle (degree)')
  subplot(1,3,2);plot(rot_vel_trace)
 title('rotation velocity (degree/s)') 
 subplot(1,3,3);plot(rot_acc_trace)
 title('rotation acceleration') 