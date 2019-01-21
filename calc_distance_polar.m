
function [Reflect_dist_vector,Reflect_vec_ID] = calc_distance_polar(Reflect_Table,Reflect_ID)
%-- calculate distance matrix between two reflectors

N_reflector=((8*length(Reflect_Table)+1)^0.5+1)/2;
z=1;
k=z;
for j=1:length(Reflect_Table)
    
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
Reflect_vec_ID;

r1=Reflect_Table(Reflect_vec_ID(j,1),1); 
r2=Reflect_Table(Reflect_vec_ID(j,2),1);
theta1=Reflect_Table(Reflect_vec_ID(j,1),2); 
theta2=Reflect_Table(Reflect_vec_ID(j,2),2); 
Reflect_dist_vector=(r1^2+r2^2-r1*r2*cos(theta1-theta2))^0.5;  % calculate the reference distance vector
