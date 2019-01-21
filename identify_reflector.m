%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [status,detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,Lidar_Table,Lidar_data)
% Lidar data: reference data
% Lidar Table: new data
% modified based on v14. need to change code for c conversion auto gen. 
% updated again Jan 15th, fixed data overflow bug when detected reflector
% less than 4.
%detected_ID=zeros; %changed from =0. GD
iii=0;
%detected_reflector=zeros(1,2); %changed from =0. GD
Lidar_Table(:,3)=Lidar_data(2,:);
% gd below code is newly added to instantiate some vectors. 
count_lidar_data = length(Lidar_data);
overkill_detected_ID_total = NaN(1,count_lidar_data);
overkill_detected_reflector_x = NaN(1,count_lidar_data);
overkill_detected_reflector_y = NaN(1,count_lidar_data);
overkill_detected_reflector_angle = NaN(1,count_lidar_data);
overkill_detected_reflector_dist = NaN(1,count_lidar_data);
overkill_raw_reflector_index = NaN(1,count_lidar_data);
%gd above code is added to facilitate autogen

for ii=2:count_lidar_data-1
    if Lidar_data(3,ii)>=amp_thres && Lidar_data(2,ii)>=dist_thres
            % detect all possible peaks
            if Lidar_data(3,ii)>Lidar_data(3,ii-1) && Lidar_data(3,ii)>Lidar_data(3,ii+1)
                iii=iii+1;
                overkill_detected_ID_total(iii)=iii;
                overkill_detected_reflector_x(overkill_detected_ID_total(iii))=Lidar_Table(ii,1);
                overkill_detected_reflector_y(overkill_detected_ID_total(iii))=Lidar_Table(ii,2);
                overkill_detected_reflector_angle(overkill_detected_ID_total(iii))=Lidar_data(1,ii);
                overkill_detected_reflector_dist(overkill_detected_ID_total(iii))=Lidar_data(2,ii);
                overkill_raw_reflector_index(iii)=ii;
                %disp('Raw detect reflector!!!');
                %disp(sprintf('Reflector ID: %i', detected_ID_total(iii)));
            end
    end
end
overkill_raw_reflector_index
% gd below code is newly added to instantiate some vectors.
detected_ID_total = overkill_detected_ID_total(1:iii);
detected_reflector_x = overkill_detected_reflector_x(1:overkill_detected_ID_total(iii));
detected_reflector_y = overkill_detected_reflector_y(1:overkill_detected_ID_total(iii));
detected_reflector_angle = overkill_detected_reflector_angle(1:overkill_detected_ID_total(iii));
detected_reflector_dist = overkill_detected_reflector_dist(1:overkill_detected_ID_total(iii));
detected_reflector_polar= NaN(iii,2);
raw_reflector_index = overkill_raw_reflector_index(1:iii);
reflector_index=NaN(1,iii);
angle_delta = NaN(1,iii);
detected_reflector_angle_g= NaN(1,iii);%this data may drop dimension in later calculation
detected_reflector = NaN(iii,2);%this data may drop dimension in later calculation
detected_ID = NaN(1,iii);%this data may drop dimension in later calculation
reflector_data = NaN(100,3); %this data may drop dimension in later calculation
angle_center_fit = NaN;
r_center_fit = NaN;
max_fit_amp = NaN;
%gd above code is added to facilitate autogen

% -- Use LSF to find the optimized reflector location
if (iii<=1)  % in case no reflector detected
    detected_reflector=zeros(1,2); %gd changed from =0
    detected_reflector_polar=zeros(1,2);%gd changed from=0
    %disp('No reflector Detected!!!');
else
%%%%--- merge all peaks from the same refletor
   %-- initilze array 1st cell
   l=1;iii=1;
   reflector_index(1)=(raw_reflector_index(1));
   detected_ID(1)=1;
   detected_reflector(1,1) = detected_reflector_x(1);
   detected_reflector(1,2) = detected_reflector_y(1);
   detected_reflector_polar(1,1) = detected_reflector_angle(1);
   detected_reflector_polar(1,2) = detected_reflector_dist(1);
   detected_reflector_angle_g(1) = detected_reflector_angle(1);
   %for ii=(l+1):length(detected_reflector_angle)
   for ii=(l+1):sum(~isnan(detected_reflector_angle)) %gd change above line to this, only count not null values. 
               angle_delta(ii)=reflector_diameter/Lidar_data(2,ii)/pi*180;
               if (abs(detected_reflector_dist(ii)-detected_reflector_dist(l))<distance_delta) && (abs(detected_reflector_angle(ii)-detected_reflector_angle(l))<angle_delta(ii))  
             % check if the detected point is from the same reflector
                 if Lidar_data(3,raw_reflector_index(ii))>=Lidar_data(3,reflector_index(iii))
                     reflector_index(iii)=raw_reflector_index(ii);
                     detected_ID(iii)=iii;
                     detected_reflector(detected_ID(iii),1) = detected_reflector_x(ii);
                     detected_reflector(detected_ID(iii),2) = detected_reflector_y(ii);
                     detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle(ii);
                     detected_reflector_polar(detected_ID(iii),1) = detected_reflector_angle(ii);%detected_reflector_polar(detected_ID(iii),1);
                     detected_reflector_polar(detected_ID(iii),2) = detected_reflector_dist(ii);%detected_reflector_polar(detected_ID(iii),2);  
                 else
                     reflector_index(iii)=reflector_index(iii);
                     detected_ID(iii)=iii;
                     detected_reflector(detected_ID(iii),1) = detected_reflector(detected_ID(iii),1);
                     detected_reflector(detected_ID(iii),2) = detected_reflector(detected_ID(iii),2);
                     detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle_g(detected_ID(iii));
                     detected_reflector_polar(detected_ID(iii),1) = detected_reflector_polar(detected_ID(iii),1);
                     detected_reflector_polar(detected_ID(iii),2) = detected_reflector_polar(detected_ID(iii),2); 
                 end    
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                iii=iii+1;
                reflector_index(iii)=(raw_reflector_index(ii));
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1) = detected_reflector_x(ii);
                detected_reflector(detected_ID(iii),2) = detected_reflector_y(ii);
                detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),1) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),2) = detected_reflector_dist(ii);
                %disp('Detect reflector!!!');
                %disp(sprintf('Reflector ID: %i', detected_ID(iii)));
               end
               l=l+1;
   end
reflector_index
   %ll=length(detected_reflector_angle_g);
   ll=sum(~isnan(detected_reflector_angle_g)); % gd changed above line to only count non null element.  
%if detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<(angle_delta(2)-360) || detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<angle_delta(2) 
   if  abs(360+detected_reflector_angle_g(1)-detected_reflector_angle_g(ll))<angle_delta(2) 
    if (abs(detected_reflector_dist(1)-detected_reflector_dist(end))<distance_delta)
   detected_reflector(1,1) = (detected_reflector(ll,1)+detected_reflector(1,1))/2;
   detected_reflector(1,2) = (detected_reflector(ll,2)+detected_reflector(1,2))/2;
   detected_reflector(ll,:) = [];
   detected_ID(ll) = [];
   detected_reflector_angle_g(ll) = [];
    end
   end
end
%if (length(detected_ID)<=1 || detected_ID(1)==0)  % in case no reflector detected
if (sum(~isnan(detected_ID))<=1 || detected_ID(1)==0)  % gd changed above line to only count non null values
    detected_reflector=zeros(1,2);%gd changed from =0
    detected_reflector_polar=zeros(1,2);%gd changed from =0
    %disp('No reflector Detected!!!');
    reflector_index = 0;
    status = 'No_Ref';
else
    status = 'Ref Found';
end

% -- Use Gaussian fit to find the optimized reflector centra location
N_fit=2;
if ref_gauss_data_fit==1
 %for ii=1:length(detected_ID)
 for ii=1:sum(~isnan(detected_ID)) %gd changed only count non null values
    for jj=1:(2*N_fit+1)
        reflector_index(ii)+jj-N_fit-1;
        reflector_data(jj,1)=Lidar_data(1,reflector_index(ii)+jj-N_fit-1);
        reflector_data(jj,2)=Lidar_data(2,reflector_index(ii)+jj-N_fit-1);
        reflector_data(jj,3)=Lidar_data(3,reflector_index(ii)+jj-N_fit-1);
    end
    reflector_data;
    [reflector_data]=check_reflector_point(reflector_data,distance_delta);
    [angle_center_fit,r_center_fit,max_fit_amp] = LSF_ref_center(reflector_data(1:jj,:));
    detected_reflector(ii,1) = cos(angle_center_fit/180*pi)*r_center_fit;
    detected_reflector(ii,2) = sin(angle_center_fit/180*pi)*r_center_fit;
    detected_reflector_angle_g(ii) = angle_center_fit;
    detected_reflector_polar(ii,1) = angle_center_fit;
    detected_reflector_polar(ii,2) = r_center_fit;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-- Below code tests the algorithm to find and merge conditional detected reflectors   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aa=[   1525
%     1524
%     1523
%     1520
%         1487
%         1484
%         1480
%         1407
%         1314
%         1303
%         12990
%         12380];
%     l=1;
%     k=1;
%     zz(1)=aa(1);
% for i=(l+1):length(aa)
%     if abs(aa(l)-aa(i))<10
%     zz(k)=(aa(l)+aa(i))/2;
%     else 
%         k=k+1;
%         zz(k)=aa(i);
%     end
%     l=l+1;
% end