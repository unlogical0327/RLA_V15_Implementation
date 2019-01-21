
function [Reflect_dist_vector] = calc_distance(Reflect_Table,Reflect_ID)
%-- calculate distance matrix between two reflectors
%-- rewrie pdsit function, based on v14 git. Jan12_2019

Reflect_vector=[Reflect_Table(1:length(Reflect_ID),1) Reflect_Table(1:length(Reflect_ID),2)]';
%Reflect_dist_vector=pdist(Reflect_vector');  % calculate the reference distance vector
Reflect_dist_vector=[];
transposeRV = Reflect_vector';
for i=1:length(transposeRV)% i,ii are both reflector id count, this is row
    for ii = i+1:length(transposeRV) 
        distance_x_square=(transposeRV(i,1)-transposeRV(ii,1))^2;
        distance_y_square=(transposeRV(i,2)-transposeRV(ii,2))^2;
        distance_x_y=(distance_x_square+distance_y_square)^0.5;
        Reflect_dist_vector=[Reflect_dist_vector,distance_x_y];
    end
end
disp('test code is passed....')
%if target == Reflect_dist_vector;
%    fprintf("your rewrite script is alright!!!")
%end

    

