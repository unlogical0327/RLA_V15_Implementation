%%%--This function do the scan for reflector map calibration
% scan once time and save the scanned reflector map as the reference
% reflector map
function [Reflector_map,Reflector_map_polar,Reflector_ID,status]=reflector_map_read(fname)
%fname = ['Data/20hz/20hz/Reflector_Map_11272018_final.txt'];
Ref_data= dlmread(fname, '\t', 0, 0);
%[calibration_data,scan_data]=PolarToRect(Reflector_map,scan_data,data_source_flag);
%[ref_status,detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
    Reflector_ID = Ref_data(:,1)';
    Reflector_map(:,1)= Ref_data(:,2);
    Reflector_map(:,2)= Ref_data(:,3);
    Reflector_map_polar(:,1)=angle(Ref_data(:,2)+Ref_data(:,3)*i)/pi*180;   % generate angle
    Reflector_map_polar(:,2)=(Ref_data(:,2).^2+Ref_data(:,3).^2).^0.5;   % generate distance
status=0;