clear;clc;close all;
cross_y = [1.2, 2.2,3.7,5.1,6.4,8];  %%%%km
cross_y =cross_y .* 1000;  %%%% m
cross_y =cross_y  - 8; 
cross_red_t = [60,65,62,55,65,67]; 
cross_green_t = [40,40,45,37,40,55]; 
croos_t = cross_red_t + cross_green_t; 
cross_starttime = [10,90,30,62,10,100]; 
offset = croos_t - cross_starttime; 
% cross_starttime = [0,0,0,0,0,0]; 
v_max = [80,80,80,80,80,80]; %%%% km/h
% v_max = v_max / 8 * 9;
v_max = v_max ./ 3.6 ; %%%m/s
v_min = [20,20,20,20,20,20]; %%%% km/h
v_min = v_min ./ 3.6; %%%m/s
simtime = 1800; 


%%%%绘图
%%%%%%%%%%%%%%%%%%%%%%%%%%%图形设置%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultfigurecolor','w')
figure(1);
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
xlabel('时间(s)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
ylabel('距离(m)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
axis([0 simtime,0 max(cross_y) + 1000]);
box on; 
for i = 1 : length(cross_y)
if cross_starttime(i) < cross_green_t(i)
h1 = line([0,cross_green_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
h2 = line([cross_green_t(i) - cross_starttime(i),croos_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
else 
h2 = line([0,croos_t(i) - cross_starttime(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
end
for k = 1 : floor((simtime - (croos_t(i) - cross_starttime(i))) / croos_t(i))
    h1 = line([croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i),croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
    h2 = line([croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i),croos_t(i) - cross_starttime(i) + k  * croos_t(i)],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
end
final_time = mod((simtime - (croos_t(i) - cross_starttime(i))), croos_t(i)); 
if final_time < cross_green_t(i)
    h1 = line([simtime - final_time ,simtime],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
else 
    h1 = line([simtime - final_time ,simtime - final_time + cross_green_t(i)],[cross_y(i), cross_y(i)],'color', 'g','linestyle','-','LineWidth',3);
    h2 = line([simtime - final_time + cross_green_t(i) ,simtime],[cross_y(i), cross_y(i)],'color', 'r','linestyle','-','LineWidth',3);
end
end
legend([h1,h2],'绿灯时间区间','红灯时间区间'); 


%%%%正向求解
for i = 1 : length(cross_y)
    %%%%%%%% 求取绿灯区间
    temp = []; 
    if cross_starttime(i) < cross_green_t(i)
        temp = [temp;0, cross_green_t(i) - cross_starttime(i);];
%     else 
%         temp = [temp; -1, -1;]; 
    end
    for k = 1 : floor((simtime - (croos_t(i) - cross_starttime(i))) / croos_t(i))
        temp = [temp; croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i),croos_t(i) - cross_starttime(i) + (k - 1) * croos_t(i) + cross_green_t(i);]; 
    end
    final_time = mod((simtime - (croos_t(i) - cross_starttime(i))), croos_t(i)); 
    if final_time < cross_green_t(i)
        temp = [temp; simtime - final_time, simtime;]; 
    else 
        temp = [temp; simtime - final_time, simtime - final_time + cross_green_t(i);]; 
    end
    %%%%%%%% 求取道路长度和速度限制的时间区间
    velLimitTime = []; 
    if i == 1
        roadLength = cross_y(i); 
        minTime = roadLength / v_max(i); 
        maxTime = roadLength / v_min(i); 
        velLimitTime= [velLimitTime;minTime, maxTime;];
    else 
        roadLength = cross_y(i) - cross_y(i - 1);
        dummy = allLimitTime{i - 1};
        for j = 1 : size(dummy, 1)
            minTime = dummy(j,1) + roadLength / v_max(i); 
            maxTime = dummy(j,1) + roadLength / v_min(i); 
            velLimitTime= [velLimitTime;minTime, maxTime;];
        end
    end
    %%%%%%%% 进行区间的合并
    velLimitTime = My_Merge(velLimitTime); 
    allLimitTime(i) = {My_Insert(temp, velLimitTime)} ; 
end



%%%%逆向反推
% dummy = allLimitTime{length(cross_y)};
% %%%%获取第一个有效区间
% ans(length(cross_y),1) =dummy(1,1); 
% ans(length(cross_y),2) =dummy(1,2); 
back_dummy = allLimitTime{length(cross_y)};
backLimitTime(i,1) = back_dummy(1,1);
backLimitTime(i,2) = back_dummy(1,2);
i = length(cross_y) - 1; 
while (i >= 1)
            back_dummy = allLimitTime{i}; %%%%前向求解中每个路口的绿波区间
            roadLength = cross_y(i + 1) - cross_y(i); 
            minTime = backLimitTime(i + 1, 1) - roadLength / v_min(i); 
            maxTime = backLimitTime(i + 1, 2) - roadLength / v_max(i); 
            min_maxTime = [minTime, maxTime];
            back_dummy = My_Insert(min_maxTime,  back_dummy);
            backLimitTime(i,1) = back_dummy(1,1);
            backLimitTime(i,2) = back_dummy(1,2);
            i = i - 1;
end

for j = 1 : length(cross_y)
    h3 = line([backLimitTime(j,1),backLimitTime(j,2)],[cross_y(j), cross_y(j)],'color', 'b','linestyle','-','LineWidth',3);
end
% legend([h1,h2,h3],'绿灯时间区间','红灯时间区间','执行时间区间'); 


%导入traci相关command
import traci.constants; 
load waitexing.mat; 
%Launch SUMO in server mode and initialize the TraCI connection
system(['sumo-gui -c ' ' C:\Users\LYP\Desktop\my_mpc\sumo_uncontrol\test.sumocfg ' '--remote-port 8873 --start&']);
traci.init(8873); %注意这里8873，端口要写在括号里
% 
% t = 0 : 600;
% v = t * 0.1 / 3.6; 

vehiclePosition = []; 
vehicleSpeed = []; %%%%%车辆行驶速度 m/s
vehicleDistance = []; %%%%%车辆行驶距离 m
% main loop. %%%%单位步长 s 
countDis = 0; 
t0 = 0; 
t = [];
targetDis = 9000; 
targetspeed = [];
prespeed = 50/3.6;
counta = [];
while countDis < targetDis
%     pause(0.5);
    traci.simulation.step();
    vehiclePosition= [vehiclePosition;traci.vehicle.getPosition('0');];
    vehicleSpeed= [vehicleSpeed;traci.vehicle.getSpeed('0');];
    vehicleDistance= [vehicleDistance;traci.vehicle.getDistance('0');];
    countDis = traci.vehicle.getDistance('0');
    speed = traci.vehicle.getSpeed('0');
    counta = [counta;speed - prespeed];
    prespeed = speed;
    speed = speed * 3.6; 
    t = [t;t0]; 
    t0 = t0 + 1; 
    targeta = interp1(v,a,speed);
    targetv = (speed + targeta)/3.6; 
    targetspeed = [targetspeed;targetv];
    traci.vehicle.setSpeed('0', targetv);  
end
traci.close();
t = 0 : (length(vehicleDistance) - 1); 
hold on; 
h3 = plot(t,vehicleDistance,'Color',[0 0 0],'LineWidth',1.5);
legend([h1,h2,h3],'绿灯时间区间','红灯时间区间','Krauss车辆跟随模型位移轨迹'); 
legend([h1,h2,h3,h4],'绿灯时间区间','红灯时间区间','执行时间区间','Krauss车辆跟随模型位移轨迹'); 



function newA = My_Merge(A)
i = 1;
newA = []; 
while i <= size(A,1)
preStart = A(i,1); 
preEnd = A(i,2); 
j = i + 1; 
while(j <= size(A,1) && A(j, 1) <= preEnd)
    preEnd = max(preEnd, A(j, 2));
    j = j + 1;
end
newA = [newA; preStart, preEnd]; 
i = j; 
end
end

function My_Insert = My_Insert(A, B)
i = 1;
j = 1; 
My_Insert = []; 
while (i <= size(A,1) && j <= size(B,1))
a1 = A(i,1); 
a2 = A(i, 2);
b1 = B(j,1); 
b2 = B(j, 2);
if(b2 >= a1 && a2 >= b1)
    My_Insert = [My_Insert; max(a1, b1), min(a2, b2);]; 
end
if b2 < a1
    j = j + 1; 
else
    i = i + 1; 
end
end
end







% %%%%%%%%%%%%%%%%%%%%%%%%%%%隶属度函数示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
% set(0,'defaultfigurecolor','w')
% figure(1);
% S(1,:) = [-3.6 -2.7 -1.7 -0.8];
% S(2,:) = [0 1 1 0];
% M(1,:) = [-1.2 -0.3 0.7 1.6];
% M(2,:) = [0 1 1 0];
% L(1,:) = [1.2 2.1 3 3.9];
% L(2,:) = [0 1 1 0];
% plot(S(1,:),S(2,:),'--','Color',[0 0 0],'LineWidth',3); hold on;
% plot(M(1,:),M(2,:),'Color',[0 0 0],'LineWidth',3); hold on;
% plot(L(1,:),L(2,:),':','Color',[0 0 0],'LineWidth',3); 
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
% ylabel('隶属度','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
% legend('较小','适中','较大');
% grid on;
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
% set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%细节图放大%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set(0,'defaultfigurecolor','w')
% figure(2);
% plot(T_mot.time,T_mot.signals.values,'LineWidth',3);
% xlabel('时间(s)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
% ylabel('电机输出转矩(N*m)','FontName','宋体','FontSize',20,'LineWidth',2,'FontWeight','bold');
% axis([4.8 10.2,0 300]);
% % %%%%%详细分析%%%%%%
% % line([8.22,8.22],[0,150],'Color',[0 0 0],'LineWidth',1.5);
% % hold on;
% % text(8,30,'x = 8.23s','FontSize',20);
% % line([8.26,8.26],[0,150],'Color',[0 0 0],'LineWidth',1.5);
% % hold on;
% % text(8.27,70,'x = 8.26s','FontSize',20);
% % line([8.96,8.96],[0,150],'Color',[0 0 0],'LineWidth',1.5);
% % hold on;
% % text(8.8,70,'x = 8.96s','FontSize',20);
% % axis([8,9.15,20,120]);
% % %%%%%详细分析%%%%%%
% grid on;
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
% set(gca,'linewidth',1.2,'FontSize',20); 
% set(gcf,'unit','centimeters','position',[5,5,19,15]);
