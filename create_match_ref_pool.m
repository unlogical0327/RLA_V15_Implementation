%%%%%%%%%%%%%%%%%%%%%%%%%
%% find M x nearest points from reflector map and fill in the pool
% -- pool_size: the size of matching reflector pool
% -- Reflect_map: the reflector map
% -- Lidar_init_xy: the initial Lidar x and y location, to find M x points
% nearest points to xy
% -- program caculates the distance and sort the distance and find first M x
% points
function [match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(pool_size,Reflector_map,Lidar_current_xy)

dist_Map=((Reflector_map(:,1)-Lidar_current_xy(1,1)).^2+(Reflector_map(:,2)-Lidar_current_xy(1,2)).^2).^0.5;
t=sort(dist_Map);
match_reflect_pool_ID=find(dist_Map<=t(pool_size),pool_size);
match_reflect_pool=Reflector_map(match_reflect_pool_ID,:);
