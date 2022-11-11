
t = 1067;


i = 1; 
while((i <= length(cross_y)) && (d >= cross_y(i)))
    i = i + 1;
end
if i == length(cross_y) + 1;
    VLimit = [v_max(i - 1),v_max(i - 1)];
elseif t < backLimitTime(i,1)
VLimit_max_1 = (cross_y(i) - d) / (backLimitTime(i,1) - t);
VLimit_max = min(VLimit_max_1, v_max(i));
VLimit_min_1 = (cross_y(i) - d) / (backLimitTime(i,2) - t);
VLimit_min  = max(VLimit_min_1,v_min(i));
VLimit = [VLimit_min,VLimit_max];
elseif t <= backLimitTime(i,2)
    VLimit_min_1 = (cross_y(i) - d) / (backLimitTime(i,2) - t);
    VLimit_min = max(VLimit_min_1,v_min(i));
    VLimit_min = min(VLimit_min_1,v_max(i)); 
%     VLimit_min = min(VLimit_min,v_max(i));
    VLimit = [VLimit_min,v_max(i)];
else
     VLimit = [0,v_min(i)];
end