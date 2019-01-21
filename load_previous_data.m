function [Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,detected_ID,detected_reflector,match_reflect_pool,match_reflect_ID,wm_detected_ID,wm_detected_reflector]=load_previous_data ...
    (Lidar_trace_p,Lidar_expect_trace_p,rotation_trace_p,Lidar_update_Table_p,detected_ID_p,detected_reflector_p,match_reflect_pool_p,match_reflect_ID_p)

            Lidar_trace=Lidar_trace_p;
            Lidar_expect_trace=Lidar_expect_trace_p;
            rotation_trace=rotation_trace_p;
            Lidar_update_Table=Lidar_update_Table_p;
            detected_ID=detected_ID_p;
            detected_reflector=detected_reflector_p;
            match_reflect_pool=match_reflect_pool_p;
            match_reflect_ID=match_reflect_ID_p;
            wm_detected_ID=match_reflect_ID';
            wm_detected_reflector=match_reflect_pool;