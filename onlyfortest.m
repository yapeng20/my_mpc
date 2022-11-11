clear;clc;close all;

%导入traci相关command
import traci.constants; 
load waitexing.mat; 
%Launch SUMO in server mode and initialize the TraCI connection
system(['sumo-gui -c ' ' C:\Users\LYP\Desktop\my_mpc\sumo_uncontrol\test.sumocfg ' '--remote-port 8873 --start&']);
traci.init(8873); %注意这里8873，端口要写在括号里
% 
% t = 0 : 600;
% v = t * 0.1 / 3.6; 
veh_mass=6000; %kg 最大总质量；整备质量6000 kg
wh_0st_rrc=423.1; % (--), 滚阻系数
wh_1st_rrc=2.324; % (--), 滚阻系数
wh_2st_rrc=0.17285; % (--), 滚阻系数

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%速度轨迹示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultAxesFontName','<宋体>');
set(0,'defaultfigurecolor','w')
figure(1);
plot(t,vehicleSpeed'.*3.6,'-','Color',[0 0 0],'LineWidth',3);
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
ylabel('速度（km/h）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
xlabel('时间（s）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
legend('Krauss车辆跟随模型速度轨迹');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
%%%%%%%%%%%%%%%%%%%%%%%%%%%位移轨迹示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultfigurecolor','w')
figure(2);
plot(t,vehicleDistance,'-','Color',[0 0 0],'LineWidth',3);
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
ylabel('位移（km）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
xlabel('时间（s）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
legend('Krauss车辆跟随模型位移轨迹');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
%%%%%%%%%%%%%%%%%%%%%%%%%%%加速度轨迹示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultfigurecolor','w')
figure(3);
plot(t,counta,'-','Color',[0 0 0],'LineWidth',3);
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
ylabel('加速度（m/s^2）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
xlabel('时间（s）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
legend('Krauss车辆跟随模型加速度轨迹');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
%%%%%%%%%%%%%%%%%%%%%%%%%%%所需驱动力示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultfigurecolor','w')
figure(4);
plot(t,targetF,'-','Color',[0 0 0],'LineWidth',3);
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
ylabel('所需驱动力（N）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
xlabel('时间（s）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
legend('Krauss车辆跟随模型驱动力轨迹');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗
%%%%%%%%%%%%%%%%%%%%%%%%%%%所需能量示意图%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'defaultfigurecolor','w')
figure(5);
plot(t,sumW,'-','Color',[0 0 0],'LineWidth',3);
% axis([-4 4.2 0 1.2]); % 坐标轴范围
% set(gca,'XTickLabel',{'x1','x2','x3','x4','x5','x6','x7','x8','x9'});
ylabel('所需能量（KJ）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
xlabel('时间（s）','FontName','宋体','FontSize',10,'LineWidth',2,'FontWeight','bold');
legend('Krauss车辆跟随模型所需能量轨迹');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
set(gca,'linewidth',1.2,'FontSize',20); % 坐标轴加粗

average_speed = sum(vehicleSpeed)/length(vehicleSpeed)*3.6;
sum_a = sum(counta.*counta);
sum_F = sum(targetF.*targetF);