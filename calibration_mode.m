%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calibration mode module
% -- this is the calibration mode module used to find the initial location
% of Lidar
% Calibration is needed to locate Lidar before Lidar starts to measure the location
%% Calibration mode:
%% 1. Load the reference reflector map from reading CVS file/manual list.
%% 2. Read distance data from Lidar.
%% 3. Identify the reflectors from data point based on the reflection amplitude, angle and distance difference.
%% 4. Match with referenced reflector table and find the current position of Robot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cali_status,Lidar_trace,rotation_trace,reflector_rmse] = calibration_mode(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,Reflector_map,Reflector_map_polar,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large,thres_angle_match,Lidar_x,Lidar_y)
% --calibration_data:  x, y coordibates with amplitude
% --scan_data:         raw Lidar data-> angle/distance/amplitude
%% Define the variables and tables here
%Lidar_x=0;    % x coordinate of Lidar in meter
%Lidar_y=0;    % y coordinate of Lidar in meter
Lidar_angle=0;    % angle of Lidar in meter
Lidar_dist=0;    % distance of Lidar in meter
theta=0;      % initialize angle and distance to generate simulated test data
dist=0;
%% 1. Read the reference reflector map and 2. the distance data from Lidar.
%% 3. Identify the reflectors from data point based on the reflection amplitude, the angle and distance difference.
% continuity. Identify the reflector from background and check if the reflector is identical.
%%% identify the reflectors from Lidar data, return detected reflector array
%[ref_status,detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
[ref_status,detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector_mex(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
detected_ID
if length(detected_ID)<3
    disp('detected reflectors smaller than 2....')
    reflector_rmse=99.99;
    Lidar_init_xy=[0 0];  % column x  y; 
    Lidar_init_polar=[0 0];   % column angle  distance;
    rotation_trace=0;
else
%% 4. Match with referenced reflector table and find the current position of Robot.
    num_detect_pool=length(detected_ID);
%% 4.a Calculate distance between any two reflectors 
    [Reflect_dist_vector] = calc_distance(Reflector_map,Reflector_ID);
    match_detected_pool=detected_reflector;
    match_detected_pool_ID=detected_ID;
    %[detect_dist_vector_pool] = calc_distance(match_detected_pool,match_detected_pool_ID);
    [detect_dist_vector_pool] = calc_distance_mex(match_detected_pool,match_detected_pool_ID);
%-- sort and index the reflectors
%    Reflect_vec_ID = index_reflector(Reflect_dist_vector);
%    detect_vec_pool_ID = index_reflector(detect_dist_vector_pool);
    Reflect_vec_ID = index_reflector_mex(Reflect_dist_vector);
    detect_vec_pool_ID = index_reflector_mex(detect_dist_vector_pool);
%% 4.a* Calculate angle between any three points
    [Reflect_angle_vector,Reflect_angle_ID] = calc_angle(Reflector_map,Reflector_ID);
    [detect_angle_vector_pool,detect_angle_pool_ID] = calc_angle(match_detected_pool,match_detected_pool_ID);
%% 4.b match detected reflectors with distance in match reflector pool and return matched point ID.
    a=1;
while a==1; 
    %[matched_reflect_ID1,matched_reflect_vec_ID1,matched_detect_ID1,matched_detect_vec_ID1,result1] = match_distance_reflector(Reflect_dist_vector,Reflect_vec_ID,detect_dist_vector,detect_vec_ID,thres_dist_large,thres_dist_match);
%-- Find min matched distance
    [matched_reflect_ID1,matched_reflect_vec_ID1,matched_detect_ID1,matched_detect_vec_ID1,result1] = match_min_distance_reflector(num_detect_pool,Reflect_dist_vector,Reflect_vec_ID,detect_dist_vector_pool,detect_vec_pool_ID,thres_dist_large,thres_dist_match);
    %[matched_reflect_ID_polar1,matched_reflect_vec_ID_polar1,matched_detect_ID_polar1,matched_detect_vec_ID_polar1,result_polar1] = match_distance_reflector(Reflect_dist_vector_polar,Reflect_vec_ID_polar,detect_dist_vector_polar,detect_vec_ID_polar,thres_dist_large,thres_dist_match);
%% 4.b* match detected reflectors with angle in match reflector pool and return matched point ID.
    %[matched_reflect_ID2,matched_reflect_angle_ID2,matched_detect_ID2,matched_detect_angle_ID2,result2] = match_angle_reflector(Reflect_angle_vector,Reflect_angle_ID,detect_angle_vector,detect_angle_ID,thres_angle_match)
    [matched_reflect_ID2,matched_reflect_angle_ID2,matched_detect_ID2,matched_detect_angle_ID2,result2] = match_min_angle_reflector(num_detect_pool,Reflect_angle_vector,Reflect_angle_ID,detect_angle_vector_pool,detect_angle_pool_ID,thres_angle_match);
    %[matched_reflect_ID_polar2,matched_reflect_angle_ID_polar2,matched_detect_ID_polar2,matched_detect_angle_ID_polar2,result_polar2] = match_angle_reflector(Reflect_angle_vector_polar,Reflect_angle_ID_polar,detect_angle_vector_polar,detect_angle_ID_polar,thres_angle_match)
    if abs(result1*result2)<=100
        match_result = 0;
    else
        match_result = 1;
    end
    if match_result==1
        num_detect_pool=num_detect_pool+1;
        [match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(num_detect_pool,detected_reflector,detected_ID,Lidar_current_xy);
        [detect_dist_vector_pool] = calc_distance(match_detected_pool,match_detected_pool_ID);
%        detect_vec_pool_ID = index_reflector(detect_dist_vector_pool);
        detect_vec_pool_ID = index_reflector_mex(detect_dist_vector_pool);
        [detect_angle_vector_pool,detect_angle_pool_ID] = calc_angle(match_detected_pool,match_detected_pool_ID);
    else
        break
        disp('Find matched pool for calibration....')
        disp('matched ref reflectors: ');
        disp(sprintf('Reflector ID(dist):-%i ', matched_reflect_ID1));
        disp(sprintf('Reflector ID(angle):-%i ', matched_reflect_ID2));
        disp('matched detected reflectors: ');
        disp(sprintf('Reflector ID(dist):-%i ', matched_detect_ID1));
        disp(sprintf('Reflector ID(angle):-%i ', matched_detect_ID2));
    end
end
if result1<100
    matched_detect_ID = matched_detect_ID1;
    matched_reflect_ID = matched_reflect_ID1;
elseif result2<100
    matched_detect_ID = matched_detect_ID2;
    matched_reflect_ID = matched_reflect_ID2;
end

    [unmatched_detected_reflector,unmatched_detect_ID] = unmatched_reflectors(detected_reflector,detected_ID,matched_detect_ID);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4.c calculate R and T with matched reflector array and detected reflector array
% find Rotation and transition of matrix A and B and Lidar initial location
% as the start point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if match_result==0

   %if length(matched_reflect_ID)==length(matched_detect_ID)
    [ret_R, ret_T, Lidar_init_xy]=locate_reflector_xy(Reflector_map,matched_reflect_ID,detected_reflector,matched_detect_ID,Lidar_x,Lidar_y)
    % Calculate reflector rmse
    [reflector_rmse]=reflector_rmse_error(ret_R,ret_T,Reflector_map,matched_reflect_ID,detected_reflector,matched_detect_ID)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Lidar_trace=Lidar_init_xy;
    rotation_trace=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;   % get rotation angle from R matrix
   %end
elseif match_result==1
    disp('Bad data and wait for another scan data');
    reflector_rmse=999.99;
    Lidar_init_xy=[0 0];
    Lidar_init_polar=[0 0];
    rotation_trace=0;
    ret_R=[0 0;0 0];
end


if reflector_rmse<10
    cali_status=0;
elseif reflector_rmse>100
    cali_status=2;
elseif reflector_rmse==999.99
    cali_status=3;
    disp('Bad data and redo calibration');
else
    cali_status=1;
end

Lidar_trace=[0 0];
Lidar_trace=Lidar_init_xy;
rotation_trace=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;
Lidar_init_polar = [0 0];
Lidar_trace_polar = Lidar_init_polar;

