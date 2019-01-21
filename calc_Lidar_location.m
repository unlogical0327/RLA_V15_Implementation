%% This program calculate the x and y of Lidar from angle and distance measurement data
function [Lidar_x,Lidar_y,rotation] = calc_Lidar_location(Lidar_trace,reflect_pool_xy,reflect_pool_ID,matched_detected_polar,matched_detect_ID,dist_err,angle_err,comp_flag)
% Calculate xy from angle data
matched_detect_ID;
matched_detected_polar;
reflect_pool_xy
for ii=1:length(matched_detect_ID)-1
    alpha(ii)=matched_detected_polar(ii+1,1)-matched_detected_polar(ii,1); % angle between A and B
end
if comp_flag==1
    for ii=1:length(reflect_pool_xy)
    dist_ref(ii)=(reflect_pool_xy(ii,1).^2+reflect_pool_xy(ii,2).^2).^0.5
    angle_ref(ii)=angle(reflect_pool_xy(ii,1)+1i*reflect_pool_xy(ii,2))/pi*180
    dist_comp(ii)=dist_ref(ii)+dist_err(ii)
    angle_comp(ii)=angle_ref(ii)+angle_err(ii)
    Xa(ii)=dist_comp(ii)*cos(angle_comp(ii)/180*pi)
    Ya(ii)=dist_comp(ii)*sin(angle_comp(ii)/180*pi)
    end
else
    Xa= reflect_pool_xy(:,1);  % x of reflector from reflector map
    Ya= reflect_pool_xy(:,2);  % y of reflector from reflector map
end

% Calculate xy from distance data
    Rad = matched_detected_polar(:,2);
%    Rad = matched_detected_polar(:,2)-dist_err';
    
    if length(Rad)>3
        calc_round=length(Rad)-3;
    elseif length(Rad)==3
        calc_round=1;
    else
        disp('No enough reflectors found')
    end
    %calc_round = floor(length(Rad)/3);
    for i=1:calc_round
%% -- solve x and y from at least 3 points intersection point
%    x_dist(i) =-(Rad(i)^2*Ya(i+1)-Rad(i+1)^2*Ya(i)-Rad(i)^2*Ya(i+2)+Rad(i+2)^2*Ya(i)+Rad(i+1)^2*Ya(i+2)-Rad(i+2)^2*Ya(i+1))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
%    y_dist(i) =(Rad(i)^2*Xa(i+1)-Rad(i+1)^2*Xa(i)-Rad(i)^2*Xa(i+2)+Rad(i+2)^2*Xa(i)+Rad(i+1)^2*Xa(i+2)-Rad(i+2)^2*Xa(i+1))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
    x_dist(i) =-(Rad(i)^2*Ya(i+1)-Rad(i+1)^2*Ya(i)-Rad(i)^2*Ya(i+2)+Rad(i+2)^2*Ya(i)+Rad(i+1)^2*Ya(i+2)-Rad(i+2)^2*Ya(i+1)-Xa(i)^2*Ya(i+1)+Xa(i+1)^2*Ya(i)+Xa(i)^2*Ya(i+2)-Xa(i+2)^2*Ya(i)-Xa(i+1)^2*Ya(i+2)+Xa(i+2)^2*Ya(i+1)+Ya(i)*Ya(i+1)^2-Ya(i)^2*Ya(i+1)-Ya(i)*Ya(i+2)^2+Ya(i)^2*Ya(i+2)+Ya(i+1)*Ya(i+2)^2-Ya(i+1)^2*Ya(i+2))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
    y_dist(i) =(Rad(i)^2*Xa(i+1)-Rad(i+1)^2*Xa(i)-Rad(i)^2*Xa(i+2)+Rad(i+2)^2*Xa(i)+Rad(i+1)^2*Xa(i+2)-Rad(i+2)^2*Xa(i+1)+Xa(i)*Xa(i+1)^2-Xa(i)^2*Xa(i+1)-Xa(i)*Xa(i+2)^2+Xa(i)^2*Xa(i+2)+Xa(i+1)*Xa(i+2)^2-Xa(i+1)^2*Xa(i+2)+Xa(i)*Ya(i+1)^2-Xa(i+1)*Ya(i)^2-Xa(i)*Ya(i+2)^2+Xa(i+2)*Ya(i)^2+Xa(i+1)*Ya(i+2)^2-Xa(i+2)*Ya(i+1)^2)/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
  
    end
    Lidar_x=mean(x_dist);
    Lidar_y=mean(y_dist);
    
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
