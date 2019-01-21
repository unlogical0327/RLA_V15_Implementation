% This program generate the combination of Nx angle from diff matrix
function [matched_reflect_angle_ID,matched_reflect_ID,matched_detect_ID,result]=gen_angle_pool(Reflect_angle_vector,Reflect_angle_ID,detect_Ref_angle_vector,detect_angle_ID,match_diff_vector,thres_angle_match)

[val,index] = min(match_diff_vector(:,1));
Ref_vec_idx=match_diff_vector(index,2);
angle_ID = Reflect_angle_ID(Ref_vec_idx,:);
det_vec_idx=match_diff_vector(index,3);
det_angle_ID = detect_angle_ID(det_vec_idx,:);
% -- find ref Q
for ii=1:length(Reflect_angle_ID)
    for jj=1:length(angle_ID)
    if Reflect_angle_ID(ii,jj)==angle_ID(1,1) || Reflect_angle_ID(ii,jj)==angle_ID(1,2) || Reflect_angle_ID(ii,jj)==angle_ID(1,3)
      angle_ID_match(ii,jj)=1;
    else
      angle_ID_match(ii,jj)=0;
  end
    end
end
angle_ID_match;
for ii=1:length(Reflect_angle_ID)
    sum_angle_ID_match(ii)=sum(angle_ID_match(ii,:));
end
matched_reflect_angle_ID=Reflect_angle_ID(find(sum_angle_ID_match==3),:);  % find right angle with right 3x points
matched_reflect_angle_vector=Reflect_angle_vector(1,find(sum_angle_ID_match==3));
matched_reflect_ID = unique(matched_reflect_angle_ID(:,2),'stable')';
%% -- find 3x points if det >3 
if length(detect_angle_ID)>3
% -- find det Q
    for ii=1:length(detect_angle_ID)
        for jj=1:length(det_angle_ID)
            if detect_angle_ID(ii,jj)==det_angle_ID(1,1) || detect_angle_ID(ii,jj)==det_angle_ID(1,2) || detect_angle_ID(ii,jj)==det_angle_ID(1,3)
                det_angle_ID_match(ii,jj)=1;
            else
                det_angle_ID_match(ii,jj)=0;
            end
        end
    end
    det_angle_ID_match;
    for ii=1:length(detect_angle_ID)
        sum_det_angle_ID_match(ii)=sum(det_angle_ID_match(ii,:));
    end
    matched_detect_angle_ID=detect_angle_ID(find(sum_det_angle_ID_match==3),:)  % find right angle with right 3x points
    matched_detect_angle_vector=detect_Ref_angle_vector(1,find(sum_det_angle_ID_match==3));
    for jj=1:length(angle_ID)
        for ii=1:length(det_angle_ID)
        if abs(matched_detect_angle_vector(1,ii)-matched_reflect_angle_vector(1,jj))<thres_angle_match
            matched_detect_angle_ID_sort(jj,:) = matched_detect_angle_ID(ii,:);
        end
        end
    end
    matched_detect_ID = unique(matched_detect_angle_ID_sort(:,2),'stable')';
else
    matched_detect_ID = unique(detect_angle_ID(:,2),'stable')';
end

if length(matched_reflect_ID) == 3 && length(matched_detect_ID) == 3
    result = 0;
else
    result=100;
end
        
        
        

