function [Lidar_trace]=gaussian_location_smooth(Lidar_trace,gaussian_step,sigma,filter_length)
       truncated_data(:,1)=Lidar_trace(end-filter_length:end,1)';
       truncated_data(:,2)=Lidar_trace(end-filter_length:end,2)';
       Lidar_x_filt = Gaussianfilter(gaussian_step, sigma,truncated_data(:,1)')';
       Lidar_y_filt = Gaussianfilter(gaussian_step, sigma,truncated_data(:,2)')';
       Lidar_xy_filt = [Lidar_x_filt(end-gaussian_step:end) Lidar_y_filt(end-gaussian_step:end)];
       Lidar_xy_filt = [Lidar_x_filt(end-gaussian_step:end) Lidar_y_filt(end-gaussian_step:end)];
        for jj=1:gaussian_step-1
            loc1=length(Lidar_trace)-jj;
            loc2=length(Lidar_xy_filt)-jj;
            Lidar_trace(loc1,:) = Lidar_xy_filt(loc2,:); 
        end