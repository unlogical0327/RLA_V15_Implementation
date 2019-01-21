function [rot_vel_trace,rot_acc_trace,radius_trace,dist_err,angle_err,ref_ID_hist,dist_err_trace,angle_err_trace,Lidar_expect_trace,matched_ref_ID_hist,matched_detect_ID_hist,rmse_trace,vel_trace,acc_trace, ...
    Lidar_trace_p,Lidar_expect_trace_p,rotation_trace_p,Lidar_update_Table_p,detected_ID_p,detected_reflector_p,match_reflect_pool_p,match_reflect_ID_p]=init_variable(num_detect_pool)
    dist_err=zeros(1,num_detect_pool);
	angle_err=zeros(1,num_detect_pool);
    ref_ID_hist = zeros(1,num_detect_pool);
    dist_err_trace = zeros(1,num_detect_pool);
    angle_err_trace = zeros(1,num_detect_pool);
    Lidar_expect_trace = [0 0];
    
    rot_vel_trace=0;
    rot_acc_trace=0;
    radius_trace=0;
    matched_ref_ID_hist=0;
    matched_detect_ID_hist=0;
    rmse_trace = 0;
    vel_trace = [0 0];
    acc_trace = [0 0];
    
    Lidar_trace_p =[0 0];
    Lidar_expect_trace_p = [0 0];
    rotation_trace_p = 0;
    Lidar_update_Table_p = 0;
    detected_ID_p = 0;
    detected_reflector_p = 0;
    match_reflect_pool_p = 0;
    match_reflect_ID_p = 0;