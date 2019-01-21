% this program use the prior information to estimate the errors generated from moving and update the
% lidar scan data to reduce the errors.
function [detected_expect_reflector,xy_vel_acc]=update_scan_data_estimation(scan_freq,detected_reflector,detected_ID,reflector_index,data_len,pose_hist)

[l,w] = size(pose_hist);
%theta_delta = 
t=1/scan_freq;
pose_hist;
x_offset1 = pose_hist(l,1)-pose_hist(l-1,1);
y_offset1 = pose_hist(l,2)-pose_hist(l-1,2);
angle_offset1 = pose_hist(l,3)-pose_hist(l-1,3);
x_offset2 = pose_hist(l-1,1)-pose_hist(l-2,1);
y_offset2 = pose_hist(l-1,2)-pose_hist(l-2,2);
angle_offset2 = pose_hist(l-1,3)-pose_hist(l-2,3);
x_vel=(x_offset1)/t;  % x velocity
y_vel=(y_offset1)/t;  % y velocity
x_acc=(x_offset1-x_offset2)/t^2;  % x acc
y_acc=(y_offset1-y_offset2)/t^2;  % y acc
pose_expect(1,3) = pose_hist(l,3)+angle_offset1; % assume the angle is not changing during the moving
pose_expect(1,1) = pose_hist(l,1)+x_offset1;
pose_expect(1,2) = pose_hist(l,2)+y_offset1;
reflector_index(1,detected_ID)/data_len
detected_expect_reflector(detected_ID,1)=detected_reflector(detected_ID,1)-x_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len);
detected_expect_reflector(detected_ID,2)=detected_reflector(detected_ID,2)-y_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len);

xy_vel_acc=[x_vel y_vel;x_acc y_acc];
%%%%%%%%%%%%%%%%%%%%%%
% Plot Lidar velocity and acceleration
