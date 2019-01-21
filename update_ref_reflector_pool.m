% This program update reflector pool with next nearest point
function [reflect_pool_xy,reflect_pool_ID,status]=update_ref_reflector_pool(pool_size_plus,num_match_pool,Reflector_map,Reflector_ID,Lidar_expect_xy,thres_near,thres_far,reflect_pool_xy,reflect_pool_ID)
dist_Map=((Reflector_map(:,1)-Lidar_expect_xy(1,1)).^2+(Reflector_map(:,2)-Lidar_expect_xy(1,2)).^2).^0.5;
l=1;
dist_Map
for ii=1:length(Reflector_map)
    if dist_Map(ii)>thres_near && dist_Map(ii)<thres_far
        filt_dist_Map(l,1)=Reflector_ID(ii);
        filt_dist_Map(l,2)=dist_Map(ii);
        l=l+1;
    end
end
t=sort(filt_dist_Map(:,2));
if pool_size_plus<length(t)
    pool_size_plus=pool_size_plus;
    status=0;
else
    pool_size_plus=t;
    disp('Avaiable Reflectors are smaller than matching pool size.....')
    status=1;
end
if status==0
    index_plus = find(filt_dist_Map(:,2)<=t(pool_size_plus),pool_size_plus)  % This is the index of original pool size +1 
    z=length(index_plus)
    num_match_pool
    index=index_plus(z-num_match_pool+1:end)
    reflect_pool_ID = filt_dist_Map(index,1);
    l=1;
    for ii=1:length(dist_Map)
        for jj=1:length(index)
        if dist_Map(ii)==filt_dist_Map(index(jj),2)
         idx(l) = ii;
         l=l+1;
        end
        end
    end
reflect_pool_xy = Reflector_map(idx,:);
elseif status==1
    disp('No reflectors updated.....')
    reflect_pool_xy=reflect_pool_xy;
    reflect_pool_ID=reflect_pool_ID;
end


