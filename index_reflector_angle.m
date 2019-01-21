
function [Reflect_angle_ID] = index_reflector_angle(Reflect_angle_vector)
% NP_enable?
% function [matched_reflect_ID] = match_reflector(match_dist_vector_pool,Reflect_vec_ID,detect_Ref_dist_vector,detect_vect_ID,thres_dist_match,NP_enable)
% Define matching threshold value here
%-- match the distance matrix with reflector tables
%%% Label distance with reference point ID
N_reflector=((8*length(Reflect_angle_vector)+1)^0.5+3)/2;  % number of points in angle array
z=1;
k=z;
w=k;
for j=1:length(Reflect_angle_vector)
    
    if k<N_reflector && w<N_reflector
        k=k;
        w=w+1;
        Reflect_vec_ID(j,1)=k;
        Reflect_vec_ID(j,2)=z;
        Reflect_vec_ID(j,3)=w;
        elseif w>=N_reflector
          k=k+1;
          w=k;
          Reflect_vec_ID(j,1)=k;
          Reflect_vec_ID(j,2)=z;
          Reflect_vec_ID(j,3)=w;
    elseif k>=N_reflector
        z=z+1;
        k=z+1;
        Reflect_vec_ID(j,1)=z;
        Reflect_vec_ID(j,2)=k;
        Reflect_vec_ID(j,3)=w;
    end
end
