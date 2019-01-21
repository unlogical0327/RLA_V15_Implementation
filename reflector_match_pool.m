%%
%%
function []= reflector_match_pool(req_update_match_pool)
%%%%%%%%%%%%%%%%%%%%%%%%%
%% find M x nearest points from reflector map and fill in the pool
% -- pool_size: the size of matching reflector pool
% -- Reflect_map: the reflector map
% -- Lidar_init_xy: the initial Lidar x and y location, to find M x points
% nearest points to xy
% -- program caculates the distance and sort the distance and find first M x
% points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load M x nearest points to matching reflector pool
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_init_xy);

