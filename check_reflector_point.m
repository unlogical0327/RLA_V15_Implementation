function [reflector_data]=check_reflector_point(reflector_data,distance_delta)
[len,wid]=size(reflector_data);
len_max=reflector_data(3,2);
for ii=1:len
    if reflector_data(ii,2)==0
        reflector_data(:,2)=len_max;
        reflector_data(:,3)=0;
    else
    if abs(reflector_data(ii,2)-len_max)>distance_delta
        diff_1=abs(reflector_data(ii,2)-len_max);
        diff_2=abs(reflector_data(len+1-ii,2)-len_max);
        if diff_1>diff_2
        reflector_data(ii,2)=reflector_data(len+1-ii,2);
        reflector_data(ii,3)=reflector_data(len+1-ii,3);
        else
        reflector_data(ii,2)=reflector_data(ii,2);
        reflector_data(ii,3)=reflector_data(ii,3);
        end
    end
    end
end