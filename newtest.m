a = '0'; 
b = '0';
if a == b
    a
end



% clear;
% clc;
% close all;
% open W.fig;
% lh = findall(gca, 'type', 'legend');% 如果图中有多条曲线，lh为一个数组
% lh = findall(gca, 'type', 'line');% 如果图中有多条曲线，lh为一个数组
% xc = get(lh, 'xdata');            % 取出x轴数据，xc是一个元胞数组
% yc = get(lh, 'ydata');            % 取出y轴数据，yc是一个元胞数组
% %如果想取得第2条曲线的x，y坐标
% x2=xc{2};
% y2=yc{2};




% targetDistance = 9000;   %%%%目标行驶距离
% 
% vehiclePosition = []; 
% vehicleSpeed = []; %%%%%车辆行驶速度 m/s
% vehicleDistance = []; %%%%%车辆行驶距离 m
% % main loop. %%%%单位步长 s 
% % for i = 1: 1000
% % %     pause(0.5);
% %     traci.simulation.step();
% %     vehiclePosition= [vehiclePosition;traci.vehicle.getPosition('0');];
% %     vehicleSpeed= [vehicleSpeed;traci.vehicle.getSpeed('0');];
% %     vehicleDistance= [vehicleDistance;traci.vehicle.getDistance('0');];
% %     if vehicleDistance(i,1) > targetDistance
% %         finalTime = i; 
% %         break;
% %     end
% % %     v = traci.vehicle.getSpeed('0');
% % %     d = traci.vehicle.getDistance('0');
% % %     if (i>5)
% % %         traci.vehicle.setLaneChangeMode('vehicle_1',0b000000000000);
% % %         traci.vehicle.setLaneChangeMode('vehicle_2',0b000000000000);
% % %         traci.vehicle.setSpeedMode('vehicle_1',00000);
% % %         traci.vehicle.setSpeedMode('vehicle_2',00000);
% % %         traci.vehicle.setSpeed('vehicle_1', 10);
% % %         traci.vehicle.setSpeed('vehicle_2', 30);
% % %     end     
% % end
% 
% 
% 
% cross_y = [1.2, 2.2,3.7,5.1,6.4,8];  %%%%km
% cross_y =cross_y .* 1000;  %%%% m
% cross_y =cross_y  - 8; 
% cross_red_t = [60,65,62,55,65,67]; 
% cross_green_t = [40,40,45,37,40,55]; 
% croos_t = cross_red_t + cross_green_t; 
% cross_starttime = [10,90,30,62,10,100]; 
% offset = croos_t - cross_starttime; 
% % cross_starttime = [0,0,0,0,0,0]; 
% v_max = [80,80,80,80,80,80,80]; %%%% km/h
% v_max = v_max ./ 3.6 ; %%%m/s
% v_min = [20,20,20,20,20,20,20]; %%%% km/h
% v_min = v_min ./ 3.6; %%%m/s
% simtime = 1800; 
% 
% 
% %%%%绘图
% %%%%%%%%%%%%%%%%%%%%%%%%%%%图形设置%%%%%%%%%%%%%%%%%%%%%%%%%%
% set(0,'defaultfigurecolor','w')
% figure(1);
% grid on;
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
% set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
% xlabel('时间(s)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
% ylabel('距离(m)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
% axis([0 simtime,0 max(cross_y) + 1000]);
% box on; 
% for i = 1 : length(cross_y)
% if cross_starttime(i) < cross_green_t(i)
% h1 = line([0,cross_green_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
% h2 = line([cross_green_t(i) - cross_starttime(i),croos_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
% else 
% h2 = line([0,croos_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
% end
% for k = 1 : floor((simtime - (croos_t(i) - cross_starttime(i))) / croos_t(i))
%     h1 = line([croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i),croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
%     h2 = line([croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i),croos_t(i) - cross_starttime(i) + k  * croos_t(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
% end
% final_time = mod((simtime - (croos_t(i) - cross_starttime(i))), croos_t(i)); 
% if final_time < cross_green_t(i)
%     h1 = line([simtime - final_time ,simtime],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
% else 
%     h1 = line([simtime - final_time ,simtime - final_time + cross_green_t(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
%     h2 = line([simtime - final_time + cross_green_t(i) ,simtime],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
% end
% end
% % legend([h1,h2],'绿灯时间区间','红灯时间区间'); 
% 
% 
% 
% %%%%正向求解
% for i = 1 : length(cross_y)
%     %%%%%%%% 求取绿灯区间
%     temp = []; 
%     if cross_starttime(i) < cross_green_t(i)
%         temp = [temp;0, cross_green_t(i) - cross_starttime(i);];
% %     else 
% %         temp = [temp; -1, -1;]; 
%     end
%     for k = 1 : floor((simtime - (croos_t(i) - cross_starttime(i))) / croos_t(i))
%         temp = [temp; croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i),croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i);]; 
%     end
%     final_time = mod((simtime - (croos_t(i) - cross_starttime(i))), croos_t(i)); 
%     if final_time < cross_green_t(i)
%         temp = [temp; simtime - final_time, simtime;]; 
%     else 
%         temp = [temp; simtime - final_time, simtime - final_time + cross_green_t(i);]; 
%     end
%     %%%%%%%% 求取道路长度和速度限制的时间区间
%     velLimitTime = []; 
%     if i == 1
%         roadLength = cross_y(i); 
%         minTime = roadLength / v_max(i); 
%         maxTime = roadLength / v_min(i); 
%         velLimitTime= [velLimitTime;minTime, maxTime;];
%     else 
%         roadLength = cross_y(i) - cross_y(i - 1);
%         dummy = allLimitTime{i - 1};
%         for j = 1 : size(dummy, 1)
%             minTime = dummy(j,1) + roadLength / v_max(i); 
%             maxTime = dummy(j,1) + roadLength / v_min(i); 
%             velLimitTime= [velLimitTime;minTime, maxTime;];
%         end
%     end
%     %%%%%%%% 进行区间的合并
%     velLimitTime = My_Merge(velLimitTime); 
%     allLimitTime(i) = {My_Insert(temp, velLimitTime)} ; 
% end
% 
% 
% 
% %%%%逆向反推
% % dummy = allLimitTime{length(cross_y)};
% % %%%%获取第一个有效区间
% % ans(length(cross_y),1) =dummy(1,1); 
% % ans(length(cross_y),2) =dummy(1,2); 
% back_dummy = allLimitTime{length(cross_y)};
% backLimitTime(i,1) = back_dummy(1,1);
% backLimitTime(i,2) = back_dummy(1,2);
% i = length(cross_y) - 1; 
% while (i >= 1)
%             back_dummy = allLimitTime{i}; %%%%前向求解中每个路口的绿波区间
%             roadLength = cross_y(i + 1) - cross_y(i); 
%             minTime = backLimitTime(i + 1, 1) - roadLength / v_min(i); 
%             maxTime = backLimitTime(i + 1, 2) - roadLength / v_max(i); 
%             min_maxTime = [minTime, maxTime]; 
%             back_dummy = My_Insert(min_maxTime,  back_dummy);
%             backLimitTime(i,1) = back_dummy(1,1);
%             backLimitTime(i,2) = back_dummy(1,2);
%             i = i - 1;
% end
% 
% for j = 1 : length(cross_y)
%     h3 = line([backLimitTime(j,1),backLimitTime(j,2)],[cross_y(j), cross_y(j)],'color', 'b','linestyle','-','LineWidth',3);
% end
% 
% for j = 1 : length(cross_y)
%     backLimitTime(j,1) = backLimitTime(j,1) - 1;
%      backLimitTime(j,2) = backLimitTime(j,2) - 1;
% %     h3 = line([backLimitTime(j,1),backLimitTime(j,2)],[cross_y(j), cross_y(j)],'color', 'b','linestyle','-','LineWidth',3);
% end
% 
% CalVelLimit(129,1192,backLimitTime,cross_y,v_max,v_min)
% 
% 
% function VLimit = CalVelLimit(t,d,backLimitTime,cross_y,v_max,v_min)
% i = 1; 
% while((i <= length(cross_y)) && (d >= cross_y(i)))
%     i = i + 1;
% end
% if i == length(cross_y) + 1;
%     VLimit = [0,v_min(i)];
% elseif t < backLimitTime(i,1)
% VLimit_max_1 = (cross_y(i) - d) / (backLimitTime(i,1) - t);
% VLimit_max = min(VLimit_max_1, v_max(i));
% VLimit_min_1 = (cross_y(i) - d) / (backLimitTime(i,2) - t);
% VLimit_min  = max(VLimit_min_1,v_min(i));
% VLimit = [VLimit_min,VLimit_max];
% elseif t < backLimitTime(i,2)
%     VLimit_min_1 = (cross_y(i) - d) / (backLimitTime(i,2) - t);
%     VLimit_min = max(VLimit_min_1,v_min(i));
%     VLimit_min = min(VLimit_min,v_max(i));
%     VLimit = [VLimit_min,v_max(i)];
% else
%      VLimit = [0,v_min(i)];
% end
% VLimit = [VLimit,i];
% end
% 
% function newA = My_Merge(A)
% i = 1;
% newA = []; 
% while i <= size(A,1)
% preStart = A(i,1); 
% preEnd = A(i,2); 
% j = i + 1; 
% while(j <= size(A,1) && A(j, 1) <= preEnd)
%     preEnd = max(preEnd, A(j, 2));
%     j = j + 1;
% end
% newA = [newA; preStart, preEnd]; 
% i = j; 
% end
% end
% 
% function My_Insert = My_Insert(A, B)
% i = 1;
% j = 1; 
% My_Insert = []; 
% while (i <= size(A,1) && j <= size(B,1))
% a1 = A(i,1); 
% a2 = A(i, 2);
% b1 = B(j,1); 
% b2 = B(j, 2);
% if(b2 >= a1 && a2 >= b1)
%     My_Insert = [My_Insert; max(a1, b1), min(a2, b2);]; 
% end
% if b2 < a1
%     j = j + 1; 
% else
%     i = i + 1; 
% end
% end
% end