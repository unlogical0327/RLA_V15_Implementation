% this function load reflector map
function [Reflector_map, Reflector_ID, status] = load_reflector_map()
fname='Reflector_map';
raw_data = dlmread( fname, ' ', 3, 0)';
for ii=1:length(raw_data)
    Reflector_ID(ii) = ii;
    Reflector_map(Reflector_ID(ii),1)=cos(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array x
    Reflector_map(Reflector_ID(ii),2)=sin(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array y
end
status=0;