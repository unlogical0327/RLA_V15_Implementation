% This program measure the Least-Square-Fit method of reflector location and reduce
% error rms
function [angle_fit,r_fit,max_fit_amp]=LSF_xy(Lidar_reflector)

Lidar_reflector = [
%-84.75	3306	76.1343
%-84.5	3305	185.951
%-84.25	3309	245.942
%-84	3313	288.491
-83.75	3318	327.351
-83.5	3311	953.974
-83.25	3303	1808.02
-83	3297	1869.42
-82.75	3312	1150.11
-82.5	3303	333.155
%-82.25	3305	293.278
%-82	3321	242.454
%-81.75	3327	151.301
];


len_r = length(Lidar_reflector);
N_r = linspace(0,len_r,len_r)';
amp_y=Lidar_reflector(:,3); % y
amp_log=log(amp_y);  % log y
p=polyfit(N_r,amp_log,2);
gauss.c = sqrt(-1/p(1));
gauss.b = -p(2)/2/p(1);
gauss.a = exp(p(3)-p(1)*gauss.b^2);

len_fit=4*len_r;
N_fit=linspace(0,len_r,len_fit);
amp_fit = gauss.a*exp(-((N_fit-gauss.b)/gauss.c).^2);
    [max_val,max_x]=max(amp_y);
    amp_y(max_x);
[max_fit_amp,max_fit_r]=max(amp_fit);
    amp_fit(max_fit_r);
delta_angle = (Lidar_reflector(end,1)-Lidar_reflector(1,1))*max_fit_r/len_fit;
r_fit=mean(Lidar_reflector(:,2));
angle_fit = Lidar_reflector(1,1)+delta_angle;

   figure(11);plot(N_r,amp_y,'*r',N_fit,amp_fit,'-b')
   title('Gaussian fitting curve of reflector amplitude')


