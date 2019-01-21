% Calculate rmse error
function [map_rmse_sort]=map_rmse_error(ret_R,ret_T,measurement_data,scan_data)
% A1 is the Lidar scanned data
% A2 is the converted map after rotation and transition
% B1 is the reference map
A1=measurement_data;
B1(1,:)=scan_data(1,:);
B1(2,:)=scan_data(2,:);
n_t=length(A1);
A2 = ret_R^-1*(A1' - repmat(ret_T, 1, n_t));   % !!!! why should I use -

theta=asin(A2(2,:)./A2(1,:));
%r=(A2(:,1).^2+A2(:,2).^2).^0.5;
[value,index]=sort(theta);
A2_sort=A2(:,index)';
theta=asin(B1(2,:)./B1(1,:));
%r=(B1(:,1).^2+B1(:,2).^2).^0.5;
[value,index]=sort(theta);
B1_sort=B1(:,index)';

err = A2_sort - B1_sort;
err = err .* err;
err = sum(err(:));
map_rmse_sort = sqrt(err/n_t);
%disp(sprintf("RMSE: %f", rmse));
%disp(sprintf('Whole map RMSE: %f', map_rmse_sort));
%disp('If RMSE is approaching zero, the matching is getting very close!');