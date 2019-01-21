%%%%Plot lidar data map, reflector and lidar trace
function []=Plot_worldmap_Lidar_center_color(Lidar_update_Table,scan_data,match_reflect_pool,matched_reflect_ID,detected_reflector,detected_ID,Lidar_trace)

figure(104)

xlabel('x(mm)')
ylabel('y(mm)')
title('World map Display')

plot(match_reflect_pool(1:length(matched_reflect_ID),1),match_reflect_pool(1:length(matched_reflect_ID),2),'+k')
color='r';
plot_reflector(detected_reflector,detected_ID',color)
c=scan_data(3,:);
scatter(Lidar_update_Table(:,1),Lidar_update_Table(:,2),10,'filled','cdata',c);
h = colorbar;
set(get(h,'label'),'string','Reflection strength');
plot(Lidar_trace(end,1),Lidar_trace(end,2),'ok')
x_display_range = 5000;
y_display_range = 5000;
xmin=Lidar_trace(end,1)-x_display_range;
xmax=Lidar_trace(end,1)+x_display_range;
ymin=Lidar_trace(end,2)-y_display_range;
ymax=Lidar_trace(end,2)+y_display_range;
axis([xmin xmax ymin ymax]); % define x and y range
hold off

grid on