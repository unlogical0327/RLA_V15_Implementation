%%%%%%%%%%%%
function [Lidar_update_Table,Lidar_update_xy,theta_rot,reflector_rmse,map_rmse]=convert_reflector_pool_world_map(measurement_data,scan_data,match_reflect_pool,matched_reflect_ID,match_detect_pool,matched_detect_ID,Lidar_x,Lidar_y);        
%[ret_R,ret_T,Lidar_update_xy]=locate_reflector_xy(match_reflect_pool,matched_reflect_ID,detected_reflector,matched_detect_ID,Lidar_x,Lidar_y);
        [ret_R,ret_T,Lidar_update_xy]=locate_reflector_pool_xy(match_reflect_pool,matched_reflect_ID,match_detect_pool,matched_detect_ID,Lidar_x,Lidar_y);
        theta_rot=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;  % find the rotation angle from Lidar 
%        if theta_rot<0
%            theta_rot=360+theta_rot;
%        end
        % Calculate reflector rmse errors
        [reflector_rmse]=reflector_pool_rmse_error(ret_R,ret_T,match_reflect_pool,matched_reflect_ID,match_detect_pool,matched_detect_ID);
        %% 2.d calculate updated map in the world map
        [Lidar_update_Table,Lidar_update_xy]=update_Lidar_scan_xy(ret_R,ret_T,measurement_data,scan_data,Lidar_x,Lidar_y);
        % calculate map rmse errors
        [map_rmse]=map_rmse_error(ret_R,ret_T,Lidar_update_Table,scan_data);