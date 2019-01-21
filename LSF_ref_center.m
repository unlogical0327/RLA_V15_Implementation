% This program measure the Least-Square-Fit method of reflector location and reduce
% error rms
function [angle_fit,r_fit,max_fit_amp]=LSF_ref_center(Lidar_reflector)
Lidar_reflector
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
[max_fit_amp,max_fit_r]=max(amp_fit);
delta_angle = (Lidar_reflector(end,1)-Lidar_reflector(1,1))*max_fit_r/len_fit;
mean_r = mean(Lidar_reflector(:,2));
[max_amp,max_idx] = max(Lidar_reflector(:,3));
max_r=Lidar_reflector(max_idx,2);   % find r at max amp
range=min(length(Lidar_reflector)-max_idx,max_idx-1);
max_r_mean=mean(Lidar_reflector(max_idx-range:max_idx+range,2));   % mean r around max amp
r_fit =max_r_mean;
if r_fit < max_r*0.8 || r_fit < max_r-400;   % check if bad data exist in mean value
    %r_fit=max_r;
    r_fit=max_r_mean;
end
angle_fit = Lidar_reflector(1,1)+delta_angle;

%    figure(11);plot(N_r,amp_y,'*r',N_fit,amp_fit,'-b')
%    title('Gaussian fitting curve of reflector amplitude')


