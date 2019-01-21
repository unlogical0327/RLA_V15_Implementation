%this program calculate angle and distance of each identified reflector
function [detected_reflector_polar,detected_ID,detected_pool_polar,detected_pool_ID,detected_dist_max,status]=generate_detected_pool(pool_size,detected_reflector_polar,detected_ID,reflector_index,bad_detected_ID,thres_near,thres_far)
%% find M x nearest points from reflector map and fill in the pool
% -- pool_size: the size of matching reflector pool
% -- Reflect_map: the reflector map
% -- Lidar_current_xy: the initial Lidar x and y location, to find M x points
% nearest points to xy
% -- program caculates the distance and sort the distance and find first M x
% Reflector_Map: xy coordinate
if bad_detected_ID~=0
    for ii=1:length(bad_detected_ID)
        for jj=1:length(detected_ID)
            if bad_detected_ID(ii)==detected_ID(jj)   
                detected_reflector_polar(jj,:)=100000;
                detected_ID(jj)=100000;
            end
        end
    end
end
det_dist_Map=detected_reflector_polar(:,2);
l=1;
for ii=1:length(detected_reflector_polar)
    if det_dist_Map(ii)>thres_near && det_dist_Map(ii)<thres_far
        filt_dist_Map(l,1)=detected_ID(ii);
        filt_dist_Map(l,2)=det_dist_Map(ii);
        filt_dist_Map(l,3)=reflector_index(ii);        
        l=l+1;
    end
end
det_t=sort(filt_dist_Map(:,2))
if pool_size<=length(det_t)
    pool_size=pool_size;
    status=0;
else
    pool_size=length(det_t);
    disp('Detected Reflectors are smaller than matching pool size.....')
    status=1;
end
if length(det_t)>3
    pool_Hi=pool_size+1;   % skip the nearest reflector if find more reflectors
    pool_Lo=2;
else
    pool_Hi=pool_size;
    pool_Lo=1;
end
l=1;
for i=1:length(det_t)
    if filt_dist_Map(i,2)<=det_t(pool_Hi) && filt_dist_Map(i,2)>=det_t(pool_Lo)
        index(l) = i;
        l=l+1;
    end
end
    index
detected_pool_ID = filt_dist_Map(index,1);
detected_dist_max = det_t(pool_Hi)
l=1;
for ii=1:length(det_dist_Map)
    for jj=1:length(index)
    if det_dist_Map(ii)==filt_dist_Map(index(jj),2)
        idx(l) = ii;
        l=l+1;
    end
    end
end
detected_pool_polar = detected_reflector_polar(idx,:)
