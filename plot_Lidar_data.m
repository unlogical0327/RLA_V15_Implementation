%%%
function []=plot_Lidar_data(measurement_data3)
measurement_data(:,1)=measurement_data3(:,1);
measurement_data(:,2)=measurement_data3(:,2);
figure(102)
%subplot(1,2,1)
xlabel('x(mm)')
ylabel('y(mm)')
title('Lidar view')
set (gcf,'Position',[50,50,600,600], 'color','w')
grid on
hold on;%plot(measurement_data(:,1),measurement_data(:,2),'.g');
plot(measurement_data(:,1),measurement_data(:,2),'.','Color',[rand(),rand(),rand()]);
color='g';
axis equal %square