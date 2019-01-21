
function [Reflect_vec_ID] = index_reflector(match_dist_vector_pool)
% Modified Jan 13th, based on v14. 
%%% Label distance with reference point ID
mdvp_length=length(match_dist_vector_pool);
N_reflector=((8*mdvp_length+1)^0.5+1)/2;
z=1;
k=z;
Reflect_vec_ID = zeros(mdvp_length,2); % define first for C code. added Jan 13th
for j=1:mdvp_length
    
    if k<N_reflector
        k=k+1;
        Reflect_vec_ID(j,1)=z;
        Reflect_vec_ID(j,2)=k;
    elseif k>=N_reflector
        z=z+1;
        k=z+1;
        Reflect_vec_ID(j,1)=z;
        Reflect_vec_ID(j,2)=k;
    end
end
disp('New pg pass!!')