% Report RMS errors of reflector array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find Rotation and transition of matrix A and B
% to find the location of Lidar itself, we calculate the centroid of many
% points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ret_R, ret_T, Lidar_update_polar]=locate_reflector_polar(Reflector_Table,Reflector_ID,detected_reflector,detected_ID,Lidar_angle,Lidar_dist)

%A1=[Reflector_Table(1:length(Reflector_ID),1) Reflector_Table(1:length(Reflector_ID),2)];
%B1=[detected_reflector(1:length(detected_ID),1) detected_reflector(1:length(detected_ID),2)];
A1=[Reflector_Table(Reflector_ID,1) Reflector_Table(Reflector_ID,2)];
detected_reflector;
detected_ID;
B1=[detected_reflector(detected_ID,1) detected_reflector(detected_ID,2)];

n_t=length(A1);
[ret_R,ret_T]=rigid_transform_2D(A1, B1);

% ret_R*A1';
% repmat(ret_T, 1, n_t);
% A2 = (ret_R*A1') + repmat(ret_T, 1, n_t);
% A2 = A2';

Lidar_polar=[Lidar_angle;Lidar_dist]';
Lidar_update_polar=(ret_R^-1*(Lidar_polar'-repmat(ret_T, 1, 1)))';  % get the x y in world coordinate
%Lidar_update_xy=(ret_R*Lidar_xy'+ret_T)'
