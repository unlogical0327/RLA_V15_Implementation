%this program calculate angle and distance of each identified reflector
function [reflect_pool_xy,reflect_pool_dist,reflect_pool_angle,reflect_pool_ID,bad_detected_ID,bad_detected_reflector_polar,dist_err,angle_err,pool_status,queue_status,match_status]= ...
    generate_reflector_pool(pool_size,Reflector_map,Reflector_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID,detected_dist_max,dist_err_trace,angle_err_trace,thres_near,thres_far,thres_dist,thres_angle)
%% find M x nearest points from reflector map and fill in the pool
% -- pool_size: the size of matching reflector pool
% -- Reflect_map: the reflector map
% -- program caculates the distance and sort the distance and find first M x
% -- Reflector_Map: xy coordinate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
dist_Map=((Reflector_map(:,1)-Lidar_expect_xy(1,1)).^2+(Reflector_map(:,2)-Lidar_expect_xy(1,2)).^2).^0.5;
reflect_angle = angle((Reflector_map(:,1)-Lidar_expect_xy(1,1))+(Reflector_map(:,2)-Lidar_expect_xy(1,2))*i)/pi*180;
l=1;
for ii=1:length(Reflector_map)
    if dist_Map(ii)>thres_near && dist_Map(ii)<thres_far
        filt_dist_Map(l,1)=Reflector_ID(ii);   % filtered reflector ID
        filt_dist_Map(l,2)=dist_Map(ii);  % filtered distance
        filt_dist_Map(l,3)=reflect_angle(ii);   % filtered angle
        filt_dist_Map(l,4)=Reflector_map(ii,1);   % filtered x
        filt_dist_Map(l,5)=Reflector_map(ii,2);   % filtered y
        l=l+1;
    end
end
%filt_dist_Map
if l==1
    disp('No fitting data is in the range....')
    reflect_pool_xy=0;
    reflect_pool_dist=0;
    reflect_pool_angle=0;
    reflect_pool_ID=0;
    bad_detected_ID=0;
    bad_detected_reflector_polar=0;
    dist_err=0;
    angle_err=0;
    pool_status=1;
    queue_status=1;
    match_status=1;
else
        t_total=sort(filt_dist_Map(:,2));
        detected_dist_max=detected_dist_max+thres_dist;
        idx=find(t_total<=detected_dist_max);
        t=t_total(idx);
        [len_t,wid_t]=size(t);
        [len_q,wid_q]=size(t_total);
        %%%% -- pool status check
        if pool_size<length(t)    % SOME reflectors become invalid may due to moved reflectors,  
            disp('Some reflectors from Map are missing.....Please check for any missing reflectors!!')
            pool_status=-1;
        elseif pool_size>length(t)  % Don't have enough reflectors and can't continue test...Need to update pool with matched reflectors
            disp('Unmatched reflectors are found.....Please update map with NEW reflectors')
            pool_status=1;
        elseif pool_size==length(t)  % ref match with det 
            disp('Detected reflectors # are matched with ref reflectors.....')
            pool_status=0;
        end
        %%%% -- queue status check
        if len_t==len_q  % ref match with det 
            disp('Ref reflector pool reach the end of ref reflector queue.....')
            queue_status=1;
        else
            queue_status=0;
        end
        %%%%%%%%%%%%%%%%%%%%%%%
            w=1;  % pool size of matched reflectors
            h=1;  % pool size of unmatched reflectors
            match_flag(1:len_t)=0;
        %%%%%%%%%%%%%%%%%%%%%%%
        Lidar_angle=mean(angle_err_trace(end))
        % -- If pool status=1, find unmatched reflectors
        if pool_status==0 || pool_status==-1 || pool_status==1  % healthy pool status to go ahead test..
         %   disp('Match detected reflectors with ref reflectors....')
            for ll=1:len_t
             for ii=1:length(t_total)
                if filt_dist_Map(ii,2)==t(ll)
                    index(ll) = ii;
                end
             end
            end
            index;
            thres_weight=thres_dist*thres_angle;
            [l_det,w_det]=size(detected_pool_polar);
            filt_dist_Map(index,:)
            for l=1:l_det   % detected pool
                for ii=1:len_t                    % ref pool
                    delta_dist(ii,l)=abs(detected_pool_polar(l,2)-filt_dist_Map(index(ii),2));%-dist_err_trace(end,l));
                    delta_angle(ii,l)=abs(detected_pool_polar(l,1)-filt_dist_Map(index(ii),3)+pose_expect);  %-pose_expect);    %-angle_err_trace(end,l));
                    if delta_angle(ii,l)>360 && delta_angle(ii,l)<360+thres_angle
                    delta_angle(ii,l)=delta_angle(ii,l)-360;   % unwrap angle more than 360
                    elseif delta_angle(ii,l)>360-thres_angle && delta_angle(ii,l)<360
                    delta_angle(ii,l)=360-delta_angle(ii,l);   % unwrap angle more than 360                        
                    end
                    weight(ii,l)=delta_dist(ii,l)*delta_angle(ii,l);
                end
                   [min_weight(l),idx_weight] = min(weight(:,l));          
               if min_weight(l)<thres_weight && match_flag(idx_weight)==0 && delta_angle(idx_weight,l)<thres_angle && delta_dist(idx_weight,l)<thres_dist 
                    reflect_pool_xy(w,1) = filt_dist_Map(index(idx_weight),4);
                    reflect_pool_xy(w,2) = filt_dist_Map(index(idx_weight),5);
                    reflect_pool_angle(w) = filt_dist_Map(index(idx_weight),3);
                    reflect_pool_dist(w) = filt_dist_Map(index(idx_weight),2);
                    reflect_pool_ID(w,1) = filt_dist_Map(index(idx_weight),1);
                    match_detected_ID(w,1) = detected_pool_ID(l,1);
                    dist_err(w) = delta_dist(idx_weight,l);   %min_dist(l);
                    angle_err(w) = delta_angle(idx_weight,l);   %min_angle(l);
                    match_flag(idx_weight)=1;  % flag to detect if this line is taken
                    w=w+1;
               else
                   detected_pool_ID(l,1)
                   h
                   bad_detected_ID(h) = detected_pool_ID(l,1);
                   bad_detected_reflector_polar(h,:) = detected_pool_polar(l,:);
                   h=h+1;
              end
            end
            if w==1
                match_detected_ID=0;
            end
            if h==1
                   bad_detected_ID(h) = 0;
                   bad_detected_reflector_polar(h,:) = 0;
            end
        %%%%%%% -- update match status!!!!    
            if w==pool_size+1 % reflector pool is full  
                disp('Reflector pool is full.....')
                delta_dist
                delta_angle
                weight
                pose_expect
                bad_detected_ID=0;  % this is used to pass bad detected reflector ID
                bad_detected_reflector_polar=0;
                match_status=0;
            elseif w<=pool_size && w>1
                disp('No enough matched reflector is found...# of unmatched reflectors:')
                num_unmatched = h-1;
                delta_dist
                delta_angle
                weight
                pose_expect
                match_detected_ID
                detected_pool_ID
                bad_detected_ID  %=setdiff(detected_pool_ID,match_detected_ID)
                bad_detected_reflector_polar;    %=detected_pool_polar(idx,:)
                match_status=1;
            elseif w==1
                disp('No matched reflector is found...')
                delta_dist
                delta_angle
                weight
                pose_expect
                match_detected_ID
                detected_pool_ID
                reflect_pool_xy=0;
                reflect_pool_dist=0;
                reflect_pool_angle=0;
                reflect_pool_ID=0;
                bad_detected_ID=0;
                bad_detected_reflector_polar=0;
                dist_err=0;
                angle_err=0;
                pool_status=1;
                queue_status=1;
                match_status=1;
            end

        end
end
