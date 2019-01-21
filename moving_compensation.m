% this porgram compensate errors induced by movement
function [Lidar_xy_com] = moving_compensation(match_reflect_pool,matched_reflect_ID,reflector_index,match_detected_pool,matched_detect_ID,dist_err,angle_err)
dist=(match_reflect_pool(matched_reflect_ID,1)^2+match_reflect_pool(matched_reflect_ID,2)^2)^0.5;






