%% This program calculate the x and y of Lidar from angle and distance measurement data
function [Lidar_x,Lidar_y,rotation] = calc_Lidar_location_w_displ(Lidar_trace,rotation_trace,reflect_pool_xy,reflect_pool_ID,matched_detected_polar,matched_detect_ID,dist_err,angle_err)
% Calculate xy from angle data
%     Lidar_trace;
%     reflect_pool_xy;
for i=1:length(reflect_pool_ID)
    z(i) = ((reflect_pool_xy(i,1)-Lidar_trace(end,1))^2+(reflect_pool_xy(i,2)-Lidar_trace(end,2))^2)^0.5;
    r(i) = matched_detected_polar(i,2);
    complex = (reflect_pool_xy(i,1)-Lidar_trace(end,1))+1i*(reflect_pool_xy(i,2)-Lidar_trace(end,2));
    A(i)=angle(complex)/pi*180;
    if A(i)>0
        A(i)=180-A(i);
    elseif A(i)<0
        A(i)=180-abs(A(i));
    end
    if matched_detected_polar(i,1)>0
        angle_i=180-matched_detected_polar(i,1);
    elseif matched_detected_polar(i,1)<0
        angle_i=180-abs(matched_detected_polar(i,1));
    end
    alpha(i) = A(i)-angle_i;
    l(i) = r(i)^2+z(i)^2-2*r(i)*z(i)*cos(alpha(i)/180*pi);
end
last_angle=matched_detected_polar(end,1);
l_total=l(end)*360/last_angle;
x_offset=cos(rotation_trace(end)/180*pi)*l_total;
y_offset=sin(rotation_trace(end)/180*pi)*l_total;
    Lidar_x=Lidar_trace(end,1)+x_offset;
    Lidar_y=Lidar_trace(end,2)+y_offset;
    
%    Lidar_x=mean(x_angle);
%    Lidar_y=mean(y_angle);  
    
    data=matched_detected_polar';
[matched_detected_xy,scan_data]=PolarToRect(data);  
% reflect_pool_xy;
% matched_detected_xy;
A1=[reflect_pool_xy(1:length(reflect_pool_ID),1) reflect_pool_xy(1:length(reflect_pool_ID),2)];
B1=[matched_detected_xy(1:length(matched_detect_ID),1) matched_detected_xy(1:length(matched_detect_ID),2)];
n_t=length(A1);
[ret_R,ret_T]=rigid_transform_2D(A1, B1);
rotation=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;  % find the rotation angle from Lidar 
