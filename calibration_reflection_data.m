% This program calibrate reflector data with measurement data
%function []=calibration_reflection_data()
% -- di_d distance
% -- 
a1 = 829;
b1 = -2514;
c1 = 4904;
a2 = 1.224e+04;
b2 = -1.376e+05;
c2 = 9.553e+04;
for i=1000:25000
    d_am_data(i-1000) = a1*exp(0-((i-b1)/c1)^2)+a2*exp(0-((i-b2)/c2)^2);
	if i~=1000
		d_am_data(i-1000)=d_am_data(i-1000)/d_am_data(0);
    end
end
d_am_data(0)=1.0;
if (di_d(j)<1000)
	am_d(j) = am_d(j)/d_am_data(0);
elseif (di_d(j)>25000)
	am_d(j) = am_d(j)/d_am_data(24000);
else
	am_d(j)=am_d(j)/d_am_data(di_d(j)-1000);
end