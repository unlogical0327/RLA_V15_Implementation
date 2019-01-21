%%%%%%%%%%%%%%%%%%%%%%%%%
%% find M x nearest points from reflector map and fill in the pool
% -- pool_size: the size of matching reflector pool
% -- Reflect_map: the reflector map
% -- Lidar_init_xy: the initial Lidar x and y location, to find M x points
% nearest points to xy
% -- program caculates the distance and sort the distance and find first M x
% points
function [match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(pool_size,detected_reflector,detected_ID, Lidar_current_xy)

dist_Map=((detected_reflector(:,1)-Lidar_current_xy(1,1)).^2+(detected_reflector(:,2)-Lidar_current_xy(1,2)).^2).^0.5;
t=sort(dist_Map);
match_detected_pool_ID=find(dist_Map<=t(pool_size),pool_size);
match_detected_pool=detected_reflector(match_detected_pool_ID,:);
