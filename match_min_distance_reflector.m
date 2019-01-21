
function [matched_reflect_ID,matched_reflect_vec_ID,matched_detect_ID,matched_detect_vec_ID,result] = match_min_distance_reflector(num_detect_pool,match_dist_vector_pool,Reflect_vec_ID,detect_Ref_dist_vector,detect_vec_ID,thres_dist_large,thres_dist_match)
% Define matching threshold value here
%-- match the distance matrix with reflector tables
%matched_reflect_vec_ID=[];
%matched_detect_vec_ID=[];
%matched_reflect_ID=[];
%matched_detect_ID=[];
m=0;n=0;
result=-2; %gd added to reflect "result" not calculated in later execution
%match_distance_flag(1:len_match_dist_vector_pool)=0;
len_detect_Ref_dist_vector=length(detect_Ref_dist_vector);
len_match_dist_vector_pool=length(match_dist_vector_pool);
idx_ref=NaN(1,len_detect_Ref_dist_vector);
bb=NaN(1,len_detect_Ref_dist_vector);
bbb=bb;
cc=bb;
ccc=bb;
%dd=NaN(2,len_detect_Ref_dist_vector);
%ddd=dd;
match_dist_flag=zeros(1,len_detect_Ref_dist_vector);
match_dist_ref_flag=zeros(len_match_dist_vector_pool,len_detect_Ref_dist_vector);
matched_detect_ID=[];
matched_reflect_ID=[];
min_det=NaN(1,len_detect_Ref_dist_vector);
diff_ref_detect=NaN(len_match_dist_vector_pool,len_detect_Ref_dist_vector);
matched_reflect_vec_ID=NaN(len_detect_Ref_dist_vector,2);
matched_detect_vec_ID=NaN(len_detect_Ref_dist_vector,2);


% -- Need to filter the distance larger than certain threshold
for j=1:len_match_dist_vector_pool   % Reference reflector
    for i=1:len_detect_Ref_dist_vector  % detetced reflector
        diff_ref_detect(j,i) = abs(match_dist_vector_pool(1,j)-detect_Ref_dist_vector(1,i));
    end
end

for i=1:len_detect_Ref_dist_vector  % detetced reflector
    [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
    if (match_dist_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_dist_match)
                match_dist_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_vec_ID(m,1)=Reflect_vec_ID(idx_ref(i),1);  %
                matched_reflect_vec_ID(m,2)=Reflect_vec_ID(idx_ref(i),2);
                matched_detect_vec_ID(n,1)=detect_vec_ID(i,1);  %
                matched_detect_vec_ID(n,2)=detect_vec_ID(i,2);
    elseif match_dist_ref_flag(idx_ref(i),i) == 1  && (diff_ref_detect(idx_ref(i),i)<thres_dist_match)
        diff_ref_detect(idx_ref(i),:)=[];
        [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
            if (match_dist_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_dist_match)
                match_dist_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_vec_ID(m,1)=Reflect_vec_ID(idx_ref(i),1);  %
                matched_reflect_vec_ID(m,2)=Reflect_vec_ID(idx_ref(i),2);
                matched_detect_vec_ID(n,1)=detect_vec_ID(i,1);  %
                matched_detect_vec_ID(n,2)=detect_vec_ID(i,2);
            end
    end
end
% need at least 3 reflectors
if  m>=3
    % find unique reflector from repeated pool in right sequence 
    bb=matched_reflect_vec_ID(:,1)';
    cc=matched_reflect_vec_ID(:,2)';
    dd=[bb cc];
    matched_reflect_ID=unique(dd,'stable');
    bbb=matched_detect_vec_ID(:,1)';
    ccc=matched_detect_vec_ID(:,2)';
    ddd=[bbb ccc];
    matched_detect_ID=unique(ddd,'stable');
   if length(matched_reflect_ID)~= length(matched_detect_ID) % added +1 for debug only
    [new_matched_reflect_vec_ID,new_matched_reflect_ID,new_matched_detect_ID]=replace_dist_matching_point(diff_ref_detect,Reflect_vec_ID,matched_reflect_vec_ID,matched_detect_vec_ID); % gd added matched_reflect_vec_ID on left side to match sub fun definition
        if length(new_matched_reflect_ID)~= length(new_matched_detect_ID)
           result=100;
        else
            if length(new_matched_detect_ID) == num_detect_pool 
                result=0;
            elseif length(new_matched_detect_ID) < num_detect_pool 
                result=-1;
            elseif length(new_matched_detect_ID) > num_detect_pool 
                result=1;
            end
        end
        % below code added by gd to transfer values from sub function.
        % found auto gen doesn't work well if input output is same
        % variable. has to create separate output and transfer new
        % calculated value to calling function as such. 
        matched_reflect_ID = [];
        matched_reflect_ID = new_matched_reflect_ID;
        matched_detect_ID = [];
        matched_detect_ID = new_matched_detect_ID;
        matched_reflect_vec_ID = [];
        matched_reflect_vec_ID = new_matched_reflect_vec_ID;
        % above code added by gd
    elseif m>length(detect_vec_ID)
        result=100;
        disp('matched reflector exceeds the number of detected reflectors');
    else
    disp('matched ref reflectors: ');
    %disp(sprintf('Reflector ID(dist):-%i ', matched_reflect_ID));
    disp('matched detected reflectors: ');
    %disp(sprintf('Reflector ID(dist):-%i ', matched_detect_ID));
    %%%%%-- Define match result value
    if length(matched_detect_ID) == num_detect_pool 
           result=0;
    elseif length(matched_detect_ID) < num_detect_pool 
           result=-1;
    elseif length(matched_detect_ID) > num_detect_pool 
           result=1;
    end
    
    end
elseif m<=2   %len_detect_Ref_dist_vector
    disp('Not enough matched reflectors found');
    result=100;
    return
end

