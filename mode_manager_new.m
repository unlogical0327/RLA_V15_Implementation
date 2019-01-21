%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RLA flow design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is the top level code to implement matlab-to-C++
% verification platform
% RLA has options to generate test vectors to verify the algorithm.
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to build the blocks for the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/18/2018
function [mode,status] = mode_manager(interrupt,scan_freq,reflector_source_flag,req_update_match_pool,num_ref_pool,num_detect_pool,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,thres_dist_match,thres_dist_large,thres_angle_match)
%% -interrupt:              interrupt from GUI console to control the Lidar computing engine
%% -reflector_source_flag:  flag to define the reflector source from GUI
%% -data_source_flag:       flag to define the data source from GUI
%% -req_update_match_pool:  request to ask match pool to update to include more reflectors
%% -Reflector_map:          load Reflector map from GUI console
%% -scan_data:              load 3D Lidar data to module
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%matched_ref_ID_hist=0;
%matched_detect_ID_hist=0;

rmse_trace = 0;
moving_estimate=0;   % flag to update detected reflector with estimated offset
ref_gauss_data_fit= 1;  % flag to enable noise cancelling algorithm
%vel_trace = zeros(10000,2);    %[0 0];
%acc_trace = zeros(10000,2);    %[0 0];
%% Load Reflector map

    %fname_moving = ['Data/3/Lidar_data.txt']; % Load moving data to test moving compensation algorithm
    fname_moving = ['Data/20hz/20hz/Lidar_data.txt']; % Load moving data to test moving compensation algorithm    
    mode=1;
    [Lidar_data,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    scan_data=Lidar_data;
    %% -- Read map from data
    %[Reflector_map,Reflector_map_polar,Reflector_ID,load_ref_map_status]=reflector_map_cali_scan(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,scan_data);
    % Read reflector map from txt file
    fname_map = ['Data/20hz/20hz/Reflector_Map_12022018_final.txt'];
    [Reflector_map,Reflector_map_polar,Reflector_ID,load_ref_map_status]=reflector_map_read(fname_map);        
    
    a=1;
    tic;
    Lidar_x=0;
    Lidar_y=0;
    
while(a==1)
    tstart_cali=tic;
    %% convert polar data to rectangle data
    [calibration_data,scan_data]=PolarToRect(Lidar_data);
    %%-- Run calibration mode: calculate initial x, y and pose 
    [cali_status,Lidar_trace,rotation_trace,reflector_rmse] = calibration_mode(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_map_polar,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large,thres_angle_match,Lidar_x,Lidar_y)
    if cali_status==0
        disp('Calibration successful! Proceed to measurement mode....')
        break
    elseif cali_status==3
        disp('Data is bad, wait for new Lidar data for new Cali!!')
    else
        disp('Calibration failed, please check Lidar data!!')
        break
    end
end
tlapsed_cali=toc(tstart_cali);  % monitor calibration running time
%% Measurement mode
%-- need to read the scan data and process the data at each scan
%measurement
    Lidar_trace_p = 0;
    Lidar_expect_trace_p = 0;
    Lidar_expect_trace = zeros(10000,2);
    rotation_trace_p = 0;
    Lidar_update_Table_p = 0;
    detected_ID_p = 0;
    detected_reflector_p = 0;
    match_reflect_pool_p = 0;
    match_reflect_ID_p = 0;
 
%-- read the meas data
    mode=2;
    [Lidar_data,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    scan_data=Lidar_data;
    dist_err=zeros(1,num_detect_pool);
	angle_err=zeros(1,num_detect_pool);
    ref_ID_hist = zeros(1,num_detect_pool);
    dist_err_trace = zeros(1,num_detect_pool);
    angle_err_trace = zeros(1,num_detect_pool);
%% Moving mode
%--Use moving mode to calculate the AGV in moving. 
% This mode will use moving estimation to reduce the errors generated
% during AGV is moving at the fast speed.
%-- read the rest of scan data
    mode=3;
    moving_thres=200;   
    rot_angle_thres=0.25;
    [Lidar_data_total,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    minTime=Inf;
    tic;
    Iter_num=1;
 for ii=3:data_round  % 87 is the bad point?
    tstart=tic;
    %--Call moving mode to calculate expected pose and location
    Lidar_data = Lidar_data_total(:,(ii-1)*data_length:ii*data_length);
    [measurement_data4,scan_data]=PolarToRect(Lidar_data);
   %%-- Plot raw data
    plot_Lidar_data(measurement_data4);
   %[moving_status,Lidar_trace,rotation_trace] = moving_mode(moving_thres,rot_angle_thres,amp_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_ID,measurement_data4,scan_data,match_reflect_pool,match_reflect_ID,reflector_index,pose_his,thres_dist_match,thres_dist_large)
    [mea_status,Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,reflector_index,detected_reflector,detected_ID,dist_err,angle_err,wm_detected_reflector,wm_detected_ID,unmatched_detected_reflector,unmatched_detect_ID,matched_ref_ID_hist,matched_detect_ID_hist,map_rmse,reflector_rmse,xy_vel_acc] = measurement_mode(Iter_num,num_ref_pool,num_detect_pool,scan_freq,Reflector_map,Reflector_ID,measurement_data4,scan_data,moving_estimate,ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,Lidar_expect_trace,rotation_trace,dist_err,angle_err,thres_dist_match,thres_dist_large,thres_angle_match,reflector_rmse,matched_ref_ID_hist,matched_detect_ID_hist);
    ii
    tlapsed_m(ii)=toc(tstart);
    minTime=min(tlapsed_m(ii),minTime);
      if mea_status==3
       disp('Cant find any matched reflector, go to the another round data......')
            Lidar_trace=Lidar_trace_p;
            Lidar_expect_trace=Lidar_expect_trace_p;
            rotation_trace=rotation_trace_p;
            Lidar_update_Table=Lidar_update_Table_p;
            detected_ID=detected_ID_p;
            detected_reflector=detected_reflector_p;
            match_reflect_pool=match_reflect_pool_p;
            match_reflect_ID=match_reflect_ID_p;
            wm_detected_ID=detected_ID_p;
            wm_detected_reflector=detected_reflector_p;
        else
            Lidar_trace_p=Lidar_trace;
            Lidar_expect_trace_p=Lidar_expect_trace;
            rotation_trace_p=rotation_trace;
            Lidar_update_Table_p=Lidar_update_Table;
            detected_ID_p=detected_ID;
            detected_reflector_p=detected_reflector;
            match_reflect_pool_p=match_reflect_pool;
            match_reflect_ID_p=match_reflect_ID;
      end
      Iter_num=ii;
  [vel_update,vel_trace,acc_trace,rmse_trace,dist_err_trace,angle_err_trace,ref_ID_hist]=update_data_trace(Iter_num,xy_vel_acc,vel_trace,acc_trace,reflector_rmse,rmse_trace,matched_ref_ID_hist,ref_ID_hist,dist_err,dist_err_trace,angle_err,angle_err_trace);
 
      if mod(ii,40)==0
            %wm_detected_ID=wm_detected_ID'
            wm_detected_reflector
            plot_lidar_xy(Lidar_trace,Lidar_expect_trace)
            plot_dist_angle_err(dist_err_trace,angle_err_trace)
            plot_running_time(tlapsed_m,rmse_trace)
            plot_vel_acc_time(vel_trace,acc_trace)
            Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,wm_detected_reflector,wm_detected_ID,Lidar_trace);
       %%%%%%%%%  
      end
 end

%%%%%%%%%%%%%%%%%  Plot time lapse

 %plot_lidar_xy(Lidar_trace,Lidar_expect_trace)
 
 %plot_dist_angle_err(dist_err_trace,angle_err_trace) 
 
 tlapsed_cali;
 minTime;
 tlapsed_m;
 rmse_trace
 %plot_running_time(tlapsed_m,rmse_trace)
 %plot_vel_acc_time(vel_trace,acc_trace)
 ref_ID_hist
 rotation_trace
 Lidar_trace
 Lidar_expect_trace
 status='done';
 %%%%%%%%%%%%%%%%%%% . Plot Velocity and Acceleration
