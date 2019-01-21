% Calculate rmse error
function [reflector_rmse_sort]=reflector_rmse_error(ret_R,ret_T,Reflector_Table,Reflector_ID,detected_reflector,detected_ID)
% A1 is the Lidar scanned data (Full Data Set )
% A2 is the converted map after rotation and transition
% B1 is the reference map
%B1=[Reflector_Table(1:length(Reflector_ID),1) Reflector_Table(1:length(Reflector_ID),2)];
%A1=[detected_reflector(1:length(detected_ID),1) detected_reflector(1:length(detected_ID),2)];
B1=[Reflector_Table(Reflector_ID,1) Reflector_Table(Reflector_ID,2)]
A1=[detected_reflector(detected_ID,1) detected_reflector(detected_ID,2)]

n_t=length(A1);
%A1'
%repmat(ret_T, 1, n_t)
A2 = ret_R^-1*(A1' - repmat(ret_T, 1, n_t));
%A2 = (ret_R*A1') + repmat(ret_T, 1, n_t);
A2 = A2';


theta=angle(A2(:,1)+1i*A2(:,2));
for ii=1:length(theta)
    if theta(ii,1)<0
        theta(ii,1)=2*pi+theta(ii,1);
    end
end
%r=(A2(:,1).^2+A2(:,2).^2).^0.5;
[value,index]=sort(theta);
A2_sort=A2(index,:);
theta=angle(B1(:,1)+1i*B1(:,2));
for ii=1:length(theta)
    if theta(ii,1)<0
        theta(ii,1)=2*pi+theta(ii,1);
    end
end
%r=(B1(:,1).^2+B1(:,2).^2).^0.5;
[value,index]=sort(theta);
B1_sort=B1(index,:);
% Find the error
% err = A2 - B1;
% err = err .* err;
% err = sum(err(:));
% rmse = sqrt(err/n_t);

err = A2_sort - B1_sort;
err = err .* err;
err = sum(err(:));
reflector_rmse_sort = sqrt(err/n_t);

%disp(sprintf("RMSE: %f", rmse));
%disp(sprintf('Reflector map RMSE: %f', reflector_rmse_sort));
%disp('If RMSE is approaching zero, the matching is getting very close!');