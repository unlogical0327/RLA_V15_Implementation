% this program use the prior information to estimate the errors generated from moving and update the
% lidar scan data to reduce the errors.
function [Lidar_expect_xy,detected_expect_reflector_polar,pose_expect,xy_vel_acc]=location_expectation(scan_freq,detected_reflector_polar,detected_ID,reflector_index,data_len,Lidar_trace,rotation_trace,dist_err_trace,angle_err_trace)
t=1/scan_freq;
N=5;
for ii=1:N
    x_offset1(ii) = Lidar_trace(end-ii+1,1)-Lidar_trace(end-ii,1);
    y_offset1(ii) = Lidar_trace(end-ii+1,2)-Lidar_trace(end-ii,2);
    angle_offset1(ii) = rotation_trace(end-ii+1)-rotation_trace(end-ii);
    x_offset2(ii) = Lidar_trace(end-N-ii+1,1)-Lidar_trace(end-N-ii,1);
    y_offset2(ii) = Lidar_trace(end-N-ii+1,2)-Lidar_trace(end-N-ii,2);
    angle_offset2(ii) = rotation_trace(end-N-ii+1)-rotation_trace(end-N-ii);
end
    x_vel=mean(x_offset1)/t  % x velocity
    y_vel=mean(y_offset1)/t  % y velocity
    x_acc=(mean(x_offset1)-mean(x_offset2))/t  % x acc
    y_acc=(mean(y_offset1)-mean(y_offset2))/t  % y acc
    rot_vel=mean(angle_offset1)/t
    rot_acc=(mean(angle_offset1)-mean(angle_offset2))/t
Lidar_expect_xy = [Lidar_trace(end,1)+mean(x_offset1) Lidar_trace(end,2)+mean(y_offset1)]
angle_err_mean=mean(angle_err_trace(end,:));
dist_err_mean=mean(dist_err_trace(end,:));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate x/y velocity from rotation velocity
 %rot_vel=(rotation_trace(end)-rotation_trace(end-1))*scan_freq;            
 %rot_acc=(rot_vel(end)-rot_vel(end-1))*scan_freq;
 delta_l=((Lidar_trace(end,1)-Lidar_trace(end-1,1))^2+(Lidar_trace(end,2)-Lidar_trace(end-1,2))^2)^0.5;
 radius=delta_l/(2*rot_vel/scan_freq/180*pi);  % radius to the rotation center
 
%   if rot_vel>2000   %radius<3000
%       rot_vel
%       w=rot_vel/180*pi
%       vel_r=radius*w
%       135
%       x_vel=vel_r*cos(rotation_trace(end)/180*pi)
%       y_vel=vel_r*sin(rotation_trace(end)/180*pi)      
%   end
%pose_expect = pose_hist+angle_offset1; % assume the angle is not changing during the moving
pose_expect = rotation_trace(end)+mean(angle_offset1);
data = detected_reflector_polar';
[detected_reflector_xy,scan_data] = PolarToRect(data);  
detected_reflector_expect(detected_ID,1) = detected_reflector_xy(detected_ID,1)-x_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len);
detected_reflector_expect(detected_ID,2) = detected_reflector_xy(detected_ID,2)-y_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len);
detected_expect_reflector_polar(detected_ID,1) = angle(detected_reflector_expect(detected_ID,1)+detected_reflector_expect(detected_ID,2)*i)/pi*180;
detected_expect_reflector_polar(detected_ID,2) = (detected_reflector_expect(detected_ID,1).^2+detected_reflector_expect(detected_ID,2).^2).^0.5;
xy_vel_acc=[x_vel y_vel;x_acc y_acc];
%%%%%%%%%%%%%%%%%%%%%%
% Plot Lidar velocity and acceleration
