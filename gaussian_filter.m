function [Lidar_x_filt,Lidar_y_filt,rotation_filt]=gaussian_filter(Lidar_trace,rotation_trace,gaussian_step)
Lidar_trace
Lidar_reflector=Lidar_trace(end-gaussian_step+1:end,:);

len_r = length(Lidar_reflector);
N_r = linspace(0,len_r,len_r)';
dist_x=Lidar_reflector(:,1) % y
dist_y=Lidar_reflector(:,2)+500 % y
dist_x_log=log(dist_x)  % log y
dist_y_log=log(dist_y)  % log y
p=polyfit(N_r,dist_x_log,2)
gauss.c = sqrt(-1/p(1));
gauss.b = -p(2)/2/p(1);
gauss.a = exp(p(3)-p(1)*gauss.b^2);
Lidar_x_filt = gauss.a*exp(-((N_r-gauss.b)/gauss.c).^2);

p=polyfit(N_r,dist_y_log,2)
gauss.c = sqrt(-1/p(1));
gauss.b = -p(2)/2/p(1);
gauss.a = exp(p(3)-p(1)*gauss.b^2);
Lidar_y_filt = gauss.a*exp(-((N_r-gauss.b)/gauss.c).^2);  

%Lidar_y_filt = 0;
rotation_filt = 0;



