% This program match detected reflector pool with ref reflector pool
function [quality,status]=match_reflector_pool(reflect_pool_xy,reflect_pool_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID)
data=detected_pool_polar';
[detected_xy,scan_data]=PolarToRect(data);  
A1=[reflect_pool_xy(1:length(reflect_pool_ID),1) reflect_pool_xy(1:length(reflect_pool_ID),2)];
B1=[detected_xy(1:length(detected_pool_ID),1) detected_xy(1:length(detected_pool_ID),2)];
n_t=length(A1);
[ret_R,ret_T]=rigid_transform_2D(A1, B1);
[reflector_rmse]=reflector_pool_rmse_error(ret_R,ret_T,reflect_pool_xy,reflect_pool_ID,detected_xy,detected_pool_ID);
quality=reflector_rmse;

status=0;
%%%%%% . CHeck R and T with rotation method
% Lidar_x=Lidar_expect_xy(1,1);
% Lidar_y=Lidar_expect_xy(1,2);
% [ret_R, ret_T, Lidar_update_xy]=locate_reflector_xy(reflect_pool_xy,reflect_pool_ID,detected_xy,detected_pool_ID,Lidar_x,Lidar_y)