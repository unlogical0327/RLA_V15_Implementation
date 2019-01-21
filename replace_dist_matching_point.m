% this program replace the nearest min in matching points 
%%%%%%%%%%%%%%%%%%%%%%%%%%
function [new_matched_reflect_vec_ID,new_matched_reflect_ID,new_matched_detect_ID]=replace_dist_matching_point(diff_ref_detect,Reflect_vec_ID,matched_reflect_vec_ID,matched_detect_vec_ID)
         min_val=min(min(diff_ref_detect));  % find next min value to match the reflectors and update 
         [idx,idy]=find(diff_ref_detect == min_val);
         new_matched_reflect_vec_ID=matched_reflect_vec_ID;
         new_matched_reflect_vec_ID(idy,1)=Reflect_vec_ID(idx,1);  %
         new_matched_reflect_vec_ID(idy,2)=Reflect_vec_ID(idx,2);
         new_matched_reflect_ID=unique(new_matched_reflect_vec_ID(:,2),'stable')';
         new_matched_detect_ID=unique(matched_detect_vec_ID(:,2),'stable')'; 