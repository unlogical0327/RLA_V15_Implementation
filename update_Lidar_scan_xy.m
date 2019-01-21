% Convert the calculated rotation and transition of Lidar to system and
% update Lidar x-y map
% report RMSE of whole Lidar map (could be large due to new coming data)
function [Lidar_update_Table,Lidar_update_xy]=update_Lidar_scan_xy(ret_R,ret_T,Lidar_Table1,Lidar_Table,Lidar_x,Lidar_y)
% ret_R: rotation matrix
% ret_T: transition matrix
% Lidar_Table1: new Lidar scan data
% Lidar_Table: reference lidar data
A1=Lidar_Table1;
B1(1,:)=Lidar_Table(1,:);
B1(2,:)=Lidar_Table(2,:);
n_t=length(A1);
%size(A1');
%size(repmat(ret_T, 1, n_t));
A2 = ret_R^-1*(A1' - repmat(ret_T, 1, n_t));   % !!!! why should I use -
%A2 = A2';
Lidar_update_Table=A2';   % convert back Lidar data with R and T
Lidar_xy=[Lidar_x;Lidar_y]';  % initial x y location to calculate current location
Lidar_update_xy=(ret_R^-1*(Lidar_xy'-repmat(ret_T, 1, 1)))';
%Lidar_update_xy=(ret_R^-1*(Lidar_xy'-repmat(ret_T, 1, 2)'))';