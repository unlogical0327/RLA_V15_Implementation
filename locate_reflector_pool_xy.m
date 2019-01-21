% Report RMS errors of reflector array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find Rotation and transition of matrix A and B
% to find the location of Lidar itself, we calculate the centroid of many
% points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ret_R, ret_T, Lidar_update_xy]=locate_reflector_pool_xy(Reflector_Table,Reflector_ID,detected_reflector,detected_ID,Lidar_x,Lidar_y)

A1=[Reflector_Table(1:length(Reflector_ID),1) Reflector_Table(1:length(Reflector_ID),2)];
B1=[detected_reflector(1:length(detected_ID),1) detected_reflector(1:length(detected_ID),2)];
%A1=[Reflector_Table(Reflector_ID,1) Reflector_Table(Reflector_ID,2)]   %this one take full data as input 
%B1=[detected_reflector(detected_ID,1) detected_reflector(detected_ID,2)]

n_t=length(A1);
[ret_R,ret_T]=rigid_transform_2D(A1, B1);

% ret_R*A1';
% repmat(ret_T, 1, n_t);
% A2 = (ret_R*A1') + repmat(ret_T, 1, n_t);
% A2 = A2';

Lidar_xy=[Lidar_x;Lidar_y]';
Lidar_update_xy=(ret_R^-1*(Lidar_xy'-repmat(ret_T, 1, 1)))';  % get the x y in world coordinate
%Lidar_update_xy=(ret_R*Lidar_xy'+ret_T)'
