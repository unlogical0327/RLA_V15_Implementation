% This program find the unmatched reflectors 
function [unmatched_detected_reflector,unmatched_detect_ID] = unmatched_reflectors(detected_reflector,detected_ID,matched_detect_ID)
l=1;
unmatched_detect_ID=0;
unmatched_detected_reflector=0;
for i=1:length(detected_ID)
    if detected_ID(1,i) ~= matched_detect_ID(1,:)
        unmatched_detected_reflector(l,1) = detected_reflector(detected_ID(1,i),1);
        unmatched_detected_reflector(l,2) = detected_reflector(detected_ID(1,i),2);
        unmatched_detect_ID(1,l) = detected_ID(1,i);
        l=l+1;
    end
end
%%%%   Plot matched reflectors
if length(unmatched_detected_reflector)>1
        disp('Found unmatched detected reflectors: ');
        disp(sprintf('detected reflector ID:-%i ', unmatched_detect_ID));
else
    disp('All detected reflectors are matched with Relfector Map!!')
end

