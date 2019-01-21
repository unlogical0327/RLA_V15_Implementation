% This program calculate the angle between multiple points
function [Reflect_angle_vector,Reflect_angle_ID] = calc_angle(Reflect_Table,Reflect_ID)
%-- calculate angle matrix between two lines
%-- if there are Nx points, there are (N-1)*(N-2)/2 angles among any three points
Reflect_vector=[Reflect_Table(1:length(Reflect_ID),1) Reflect_Table(1:length(Reflect_ID),2)]';
Reflect_dist_vector=pdist(Reflect_vector');  % calculate the reference distance vector
Reflect_vec_ID = index_reflector(Reflect_dist_vector);
N=length(Reflect_ID);
NNN=N*(N-1)*(N-2)/2;
ll=0;
for ii=1:N  % center point
    for jj=1:N  % left point
        for kk=1:N   % right point
       if (ii~=jj) && (ii~=kk) && (jj<kk)
            ll=ll+1;
       Reflect_angle_ID(ll,2)=ii;        
       Reflect_angle_ID(ll,1)=jj;
       Reflect_angle_ID(ll,3)=kk;
       end
        end
    end
end
[Reflect_dist_vector] = calc_distance(Reflect_Table,Reflect_ID);
[Reflect_vec_ID] = index_reflector(Reflect_dist_vector);

for ii=1:length(Reflect_angle_ID)
   for jj=1:length(Reflect_vec_ID)
    if Reflect_angle_ID(ii,1) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,1) == Reflect_vec_ID(jj,2) %calculate a line
    if Reflect_angle_ID(ii,2) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,2) == Reflect_vec_ID(jj,2)
       a2(ii) = Reflect_dist_vector(1,jj).^2;
       a(ii) = Reflect_dist_vector(1,jj);
    end
    end
    if Reflect_angle_ID(ii,2) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,2) == Reflect_vec_ID(jj,2) %calculate b line
    if Reflect_angle_ID(ii,3) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,3) == Reflect_vec_ID(jj,2)
       b2(ii) = Reflect_dist_vector(1,jj).^2;
       b(ii) = Reflect_dist_vector(1,jj);
    end
    end
    if Reflect_angle_ID(ii,1) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,1) == Reflect_vec_ID(jj,2) %calculate b line
    if Reflect_angle_ID(ii,3) == Reflect_vec_ID(jj,1) || Reflect_angle_ID(ii,3) == Reflect_vec_ID(jj,2)
       c2(ii) = Reflect_dist_vector(1,jj).^2;
    end
    end
   end
   pos(ii) = (a2(ii)+b2(ii)-c2(ii))/(2*a(ii)*b(ii));  %cos value for each possible angle
   angle_radian(ii) = acos(pos(ii));         %cos to radian
   Reflect_angle_vector(ii)= angle_radian(ii)*180/pi;   %radian to degree
end