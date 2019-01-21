% THis program update the trace data
function [vel_update,vel_trace,acc_trace,rmse_trace,dist_err_trace,angle_err_trace,rot_vel_trace,rot_acc_trace,radius_trace,ref_ID_hist]=update_data_trace(Iter_num,scan_freq,xy_vel_acc,vel_trace,acc_trace,reflector_rmse,rmse_trace,matched_ref_ID_hist,ref_ID_hist,dist_err,dist_err_trace,angle_err,angle_err_trace,Lidar_trace,rotation_trace,rot_vel_trace,rot_acc_trace,radius_trace)
            if length(xy_vel_acc)<2 
                vel_update = [0 0];
                acc_update = [0 0];
                xy_vel_acc= [0 0;0 0];
            else
            vel_update = [xy_vel_acc(1,1) xy_vel_acc(1,2)];
            acc_update = [xy_vel_acc(2,1) xy_vel_acc(2,2)];
            end
            vel_trace(Iter_num,:)=[vel_update];
            acc_trace(Iter_num,:)=[acc_update];
            rmse_trace(Iter_num,:)=[reflector_rmse];
            [l_ref,w_ref]=size(matched_ref_ID_hist);
            if length(dist_err)<w_ref || length(angle_err)<w_ref
                dist_err=dist_err_trace(end,:);
                angle_err=angle_err_trace(end,:);
            end
            dist_err_trace(Iter_num,:)=[dist_err];
            angle_err_trace(Iter_num,:)=[angle_err];
            rot_vel_trace(Iter_num)=[(rotation_trace(end)-rotation_trace(end-1))*scan_freq];            
            if length(rot_vel_trace)>2
                rot_acc_trace(Iter_num) = [(rot_vel_trace(end)-rot_vel_trace(end-1))*scan_freq];
            else
                 rot_acc_trace(Iter_num)= 0;
            end
            delta_l=((Lidar_trace(end,1)-Lidar_trace(end-1,1))^2+(Lidar_trace(end,2)-Lidar_trace(end-1,2))^2)^0.5;
            radius_current=delta_l/(rot_vel_trace(Iter_num)/scan_freq);
            radius_trace(Iter_num)=[radius_current];
            %pose_hist=[pose_hist;rotation_trace];
            if l_ref>w_ref
                matched_ref_ID_hist=matched_ref_ID_hist';
            end
            ref_ID_hist(Iter_num,:)=[matched_ref_ID_hist];