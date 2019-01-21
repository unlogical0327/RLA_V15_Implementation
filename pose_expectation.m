% function to generate pose expectation from the previous moment pose information
% pose_expectation and pose_current are 3x1 matrix contains x, y and theta  
function [pose_expect]=pose_expectation(pose_his)
[l,w] = size(pose_his);
%theta_delta = 
t=1/20;
pose_his
x_offset1 = pose_his(l,1)-pose_his(l-1,1);
y_offset1 = pose_his(l,2)-pose_his(l-1,2);
angle_offset1 = pose_his(l,3)-pose_his(l-1,3);
x_offset2 = pose_his(l-1,1)-pose_his(l-2,1);
y_offset2 = pose_his(l-1,2)-pose_his(l-2,2);
angle_offset2 = pose_his(l-1,3)-pose_his(l-2,3);
x_vel=(x_offset1-x_offset2)/t;  % x velocity
y_vel=(y_offset1-y_offset2)/t;  % y velocity
pose_expect(1,3) = pose_his(l,3)+angle_offset1; % assume the angle is not changing during the moving
pose_expect(1,1) = pose_his(l,1)+x_offset1;
pose_expect(1,2) = pose_his(l,2)+y_offset1;

