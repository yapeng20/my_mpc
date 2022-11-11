clear;
close('all');
clc;

%导入traci相关command
import traci.constants; 

%Launch SUMO in server mode and initialize the TraCI connection
system(['sumo-gui -c ' ' C:\Users\LYP\Desktop\my_mpc\sumo_paper\test.sumocfg ' '--remote-port 8873 --start&']);
traci.init(8873); %注意这里8873，端口要写在括号里


load backLimitTime;

%%%%仿真参数设置
targetDistance = 9000;   %%%%目标行驶距离

vehiclePosition = []; 
vehicleSpeed = []; %%%%%车辆行驶速度 m/s
vehicleDistance = []; %%%%%车辆行驶距离 m
% main loop. %%%%单位步长 s 
% for i = 1: 1000
% %     pause(0.5);
%     traci.simulation.step();
%     vehiclePosition= [vehiclePosition;traci.vehicle.getPosition('0');];
%     vehicleSpeed= [vehicleSpeed;traci.vehicle.getSpeed('0');];
%     vehicleDistance= [vehicleDistance;traci.vehicle.getDistance('0');];
%     if vehicleDistance(i,1) > targetDistance
%         finalTime = i; 
%         break;
%     end
% %     v = traci.vehicle.getSpeed('0');
% %     d = traci.vehicle.getDistance('0');
% %     if (i>5)
% %         traci.vehicle.setLaneChangeMode('vehicle_1',0b000000000000);
% %         traci.vehicle.setLaneChangeMode('vehicle_2',0b000000000000);
% %         traci.vehicle.setSpeedMode('vehicle_1',00000);
% %         traci.vehicle.setSpeedMode('vehicle_2',00000);
% %         traci.vehicle.setSpeed('vehicle_1', 10);
% %         traci.vehicle.setSpeed('vehicle_2', 30);
% %     end     
% end



cross_y = [1.2, 2.2,3.7,5.1,6.4,8];  %%%%km
cross_y =cross_y .* 1000;  %%%% m
cross_y =cross_y  - 8; 
cross_red_t = [60,65,62,55,65,67]; 
cross_green_t = [40,40,45,37,40,55]; 
croos_t = cross_red_t + cross_green_t; 
cross_starttime = [10,90,30,62,10,100]; 
offset = croos_t - cross_starttime; 
% cross_starttime = [0,0,0,0,0,0]; 
v_max = [80,80,80,80,80,80,80]; %%%% km/h
v_max = v_max ./ 3.6 ; %%%m/s
v_min = [20,20,20,20,20,20,80]; %%%% km/h
v_min = v_min ./ 3.6; %%%m/s
simtime = 1800; 


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
%     backLimitTime(j,1) = backLimitTime(j,1) + 1;
%      backLimitTime(j,2) = backLimitTime(j,2) - 1;
% %     h3 = line([backLimitTime(j,1),backLimitTime(j,2)],[cross_y(j), cross_y(j)],'color', 'b','linestyle','-','LineWidth',3);
% end



m = 6000;
f0 = 423.1;
f1 = 2.324;
f2 = 0.17285;
g = 9.8; 

veh_mass=6000; %kg 最大总质量；整备质量6000 kg
wh_0st_rrc=423.1; % (--), 滚阻系数
wh_1st_rrc=2.324; % (--), 滚阻系数
wh_2st_rrc=0.17285; % (--), 滚阻系数


import casadi.*

T = 1; % sampling time [s]
N = 10; % prediction horizon
rob_diam = 0.3;

% v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

d = SX.sym('d'); v = SX.sym('v'); 
states = [d;v]; n_states = length(states);

Ft = SX.sym('Ft');
controls = [Ft]; n_controls = length(controls);
rhs = [v;(Ft-(f0+f1*v*3.6+f2*v*v*3.6*3.6))/m]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial and the reference state of the robot)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:n_states); % initial state
for k = 1:N
    st = X(:,k);  con = U(:,k);
    f_value  = f(st,con);
    st_next  = st+ (T*f_value);
    X(:,k+1) = st_next;
end
% this function to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});  

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(2,2); Q(1,1) = 1;Q(2,2) = 0.8; % weighing matrices (states)
% R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)
R = 0.5;
% compute objective
for k=1:N
%     st = X(:,k);  
    con = U(:,k);
    % calculate obj 目标函数 暂时只考虑驱动力最小
    obj = obj+ con'*R*con; 
%     if con(1,1) > 0
%     obj = obj+ con'*R*con; 
%     end
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state d
    g = [g ; X(2,k)];   %state v
end

% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = 0;  % lower bound of the states d and v
args.ubg = 200;   % upper bound of the states d and v 

% input constraints
% args.lbx(1:2:2*N-1,1) = v_min; args.lbx(2:2:2*N,1)   = omega_min;
% args.ubx(1:2:2*N-1,1) = v_max; args.ubx(2:2:2*N,1)   = omega_max;
args.lbx = -24000;
args.ubx = 8000;


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [0 ; 50/3.6 ];    % initial condition.
xs = [1.5 ; 1.5 ]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t=[];

u0 = zeros(N,n_controls);  % two control inputs 

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
countDis = 0; 
countVlimit = [];
display = []; 
prespeed = 50/3.6;
counta = [];
weight_time = 0.7; 
while(countDis < targetDistance)
    
   %%%%% 循环里面的x代表控制量，g设置为状态量
    traci.simulation.step();
    vehiclePosition= [vehiclePosition;traci.vehicle.getPosition('0');];
    vehicleSpeed= [vehicleSpeed;traci.vehicle.getSpeed('0');];
    vehicleDistance= [vehicleDistance;traci.vehicle.getDistance('0');];
    d = traci.vehicle.getDistance('0'); 
    countDis = d; 
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',N,1); % initial value of the optimization variables 控制量
%     for i = 4:2:2*N + 2
%         VLimit = CalVelLimit(t0+2*i, d,backLimitTime,cross_y,v_max,v_min)
%     end
    VLimit = CalVelLimit(t0, d,backLimitTime,cross_y,v_max,v_min);
    display = [display;t0,d,VLimit(1,1),VLimit(1,2),traci.vehicle.getSpeed('0')];
    countVlimit = [countVlimit;VLimit];
%     if VLimit(1,1) == VLimit(1,2)
%     args.lbg(2:2:2*N + 2,1) = VLimit(1,1);
%     else
%     args.lbg(2:2:2*N + 2,1) = VLimit(1,1)+(VLimit(1,2) - VLimit(1,1))*weight_time;
%     end
    args.lbg(2:2:2*N + 2,1) = VLimit(1,1)+(VLimit(1,2) - VLimit(1,1))*weight_time;
    args.ubg(2:2:2*N + 2 ,1) = VLimit(1,2);
    args.lbg(1:2:2*N + 1,1) = 0;
    args.ubg(1:2:2*N + 1,1) = 20000;
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',1,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:2,mpciter+1)= full(ff_value)';
    
    u_cl= [u_cl ; u(1,:)];
    t = [t;t0];
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    mpciter;
    mpciter = mpciter + 1;
    if d < 8000
        traci.vehicle.setSpeed('0', x0(2,1));
    else
       traci.vehicle.setSpeed('0', v_max(end)); 
    end

%     t = [t;t0];
%     t0 = t0 + 1;
    
    speed = traci.vehicle.getSpeed('0'); %%%%这里其实不对，此步长的力应该为此步长的需求速度减去刺客的速度
    a = speed - prespeed;
    prespeed = speed;
    counta =[counta;a]; 
end;
traci.close();

Fr=wh_0st_rrc + wh_1st_rrc * 3.6 * vehicleSpeed + wh_2st_rrc * (vehicleSpeed*3.6).^2;
targetF = Fr + counta * veh_mass;
sumW = [];
for i = 1 : length(targetF)
    tempW = targetF(i)*vehicleSpeed(i)/1000;
    if targetF(i) > 0
         
    else 
         tempW = tempW * 0.5;
    end
    if i == 1
        nowW = tempW;
    else
        nowW = tempW + sumW(i - 1);
    end
    sumW = [sumW;nowW];
end

%%%%%%%绘制位移图1
open light.fig;
% legend on;
hold on; 
h4 = plot(t,vehicleDistance,'Color',[0 0 1],'LineWidth',1.5);
hold off;
% hold on; 
% h5 = plot(t,countVlimit(:,1),'Color',[1 0 0],'LineWidth',1.5);
% h6 = plot(t,countVlimit(:,2),'Color',[0 1 0],'LineWidth',1.5);
% legend('车辆规划轨迹');
% legend([h1,h2,h3,h4],'绿灯时间区间','红灯时间区间','Krauss车辆跟随模型位移轨迹4','MPC速度规划位移轨迹'); 
% legend([h4],'MPC速度规划位移轨迹'); 
%%%%%%%绘制位移图2
open distance.fig;
hold on; 
plot(t,vehicleDistance,'-','Color',[1 0 0],'LineWidth',3);
%%%%%%%绘制速度图
open speed.fig;
hold on; 
plot(t,vehicleSpeed'.*3.6,'-','Color',[1 0 0],'LineWidth',3);
%%%%%%%绘制加速度图
open a.fig;
hold on; 
plot(t,counta,'-','Color',[1 0 0],'LineWidth',3);
%%%%%%%绘制需求扭矩图
open F.fig;
hold on; 
plot(t,targetF,'-','Color',[1 0 0],'LineWidth',3);
%%%%%%%绘制需求能量图
open W.fig;
hold on; 
plot(t,sumW,'-','Color',[1 0 0],'LineWidth',3);

%%%%数据后处理
average_speed = sum(vehicleSpeed)/length(vehicleSpeed)*3.6;
sum_a = sum(counta.*counta);
sum_F = sum(targetF.*targetF);


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

function VLimit = CalVelLimit(t,d,backLimitTime,cross_y,v_max,v_min)
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
%     VLimit_min = min(VLimit_min,v_max(i));
    VLimit = [VLimit_min,v_max(i)];
else
     VLimit = [0,v_min(i)];
end
end