% this program match reflectors with calculated angles abd return the matched reflector x/y and reflector ID 
function [matched_reflect_ID,matched_reflect_angle_ID,matched_detect_ID,matched_detect_angle_ID,result] = match_min_angle_reflector(num_detect_pool,Reflect_angle_vector,Reflect_angle_ID,detect_Ref_angle_vector,detect_angle_ID,thres_angle_match)
% Define matching threshold value here
%-- match the angle matrix with reflector tables
matched_reflect_angle_ID=0;
matched_detect_angle_ID=0;
matched_reflect_ID=0;
matched_detect_ID=0;
m=0;n=0;
match_angle_flag(1:length(detect_Ref_angle_vector))=0;
match_angle_ref_flag(1:length(Reflect_angle_vector),1:length(detect_Ref_angle_vector))=0;
match_diff_vector = 0;
thres_angle_Lo = 5;   % threshold to filter out the small angle
thres_angle_Hi = 175;   % threshold to filter out the large angle
thres_angle_med_Hi = 62;   % threshold to filter out the small angle
thres_angle_med_Lo = 58;   % threshold to filter out the large angle
% -- Need to filter the distance larger than certain threshold
for j=1:length(Reflect_angle_vector)   % Reference reflector
    for i=1:length(detect_Ref_angle_vector)  % detetced reflector
        if (Reflect_angle_vector(1,j)>thres_angle_Lo) && (Reflect_angle_vector(1,j)<thres_angle_Hi)
            %if (Reflect_angle_vector(1,j)>thres_angle_med_Hi) && (Reflect_angle_vector(1,j)<thres_angle_med_Lo)
                diff_ref_detect(j,i) = abs(Reflect_angle_vector(1,j)-detect_Ref_angle_vector(1,i));
            else
            diff_ref_detect(j,i) = inf;
            end
        %end
    end
end

for i=1:length(detect_Ref_angle_vector)  % detetced reflector
    [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
    if (match_angle_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
                match_angle_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_angle_ID(m,1)=Reflect_angle_ID(idx_ref(i),1);  %
                matched_reflect_angle_ID(m,2)=Reflect_angle_ID(idx_ref(i),2);
                matched_reflect_angle_ID(m,3)=Reflect_angle_ID(idx_ref(i),3);
                matched_detect_angle_ID(n,1)=detect_angle_ID(i,1);  %
                matched_detect_angle_ID(n,2)=detect_angle_ID(i,2);
                matched_detect_angle_ID(n,3)=detect_angle_ID(i,3);
                match_diff_vector(n,1)=min_det(i);
                match_diff_vector(n,2)=idx_ref(i);
                match_diff_vector(n,3)=i;
                diff_ref_detect(idx_ref(i),i)=inf;
    elseif match_angle_ref_flag(idx_ref(i),i) == 1  && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
        diff_ref_detect(idx_ref(i),:)=inf;
        [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
            if (match_angle_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
                match_angle_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_angle_ID(m,1)=Reflect_angle_ID(idx_ref(i),1);  %
                matched_reflect_angle_ID(m,2)=Reflect_angle_ID(idx_ref(i),2);
                matched_reflect_angle_ID(m,3)=Reflect_angle_ID(idx_ref(i),3);
                matched_detect_angle_ID(n,1)=detect_angle_ID(i,1);  %
                matched_detect_angle_ID(n,2)=detect_angle_ID(i,2);
                matched_detect_angle_ID(n,3)=detect_angle_ID(i,3);
                match_diff_vector(n,1)=min_det(i);  % diff min value
                match_diff_vector(n,2)=idx_ref(i);  % diff min index
                match_diff_vector(n,3)=i;
            end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if  m>=3
    % check if angle vector contain wrong points
     matched_reflect_point=unique(matched_reflect_angle_ID,'stable')';
     matched_detect_point=unique(matched_detect_angle_ID,'stable')';
     if (length(matched_reflect_point)~= length(matched_detect_point)) %&& length(detect_Ref_angle_vector)<=3
     %[matched_reflect_ID,matched_detect_ID,result]=gen_comb_angle(diff_ref_detect,Reflect_angle_ID,matched_reflect_angle_ID,matched_detect_angle_ID)
     [matched_reflect_angle_ID,matched_reflect_ID,matched_detect_ID,result]=gen_angle_pool(Reflect_angle_vector,Reflect_angle_ID,detect_Ref_angle_vector,detect_angle_ID,match_diff_vector,thres_angle_match)
     else  % extract points in order
     matched_reflect_ID=unique(matched_reflect_angle_ID(:,2),'stable')';
     matched_detect_ID=unique(matched_detect_angle_ID(:,2),'stable')';
    %%%%%-- Define match result value
        if length(matched_detect_ID) == num_detect_pool 
                result=0;
        elseif length(matched_detect_ID) < num_detect_pool 
                result=-1;
        elseif length(matched_detect_ID) > num_detect_pool 
                result=1;
        end
    
     end
elseif m>length(detect_angle_ID)
        result=100;
        disp('matched reflector exceeds the number of detected reflectors');
elseif m<=2   %length(detect_Ref_angle_vector)
    disp('Not enough matched reflectors found');
    result=100;
else
        disp('matched ref reflectors: ');
        disp(sprintf('Reflector ID(angle):-%i ', matched_reflect_ID));
        disp('matched detected reflectors: ');
        disp(sprintf('Reflector ID(angle):-%i ', matched_detect_ID));
    %%%%%-- Define match result value
        if length(matched_detect_ID) == num_detect_pool 
                result=0;
        elseif length(matched_detect_ID) < num_detect_pool 
                result=-1;
        elseif length(matched_detect_ID) > num_detect_pool 
                result=1;
        end
    
    %end
    %return
end
