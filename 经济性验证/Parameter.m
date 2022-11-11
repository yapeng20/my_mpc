clc,clear,close all; 
set(0,'defaultfigurecolor','w');

%% ��������
% load  'CWTVC.mat';
load  'control.mat';
% load  'nocontrol.mat';
cyc_t=CWTVC(:,1); % ����ʱ��
cyc_t=cyc_t';
cyc_v=CWTVC(:,2); % km/h
cyc_v=cyc_v';
%% ����������ϵͳ����
veh_mass = 6000; %kg
veh_gravity=9.8;%m/s^2   
veh_air_density=1.2258;
veh_CD=0.61;
veh_FA=3.6;%m^2
wh_1st_rrc=0.015;
wh_2st_rrc=0.000000; % 
wh_radius=0.376; %m
b=1.1; %��ת��������ϵ��
eff_wheel=1;
eff_fd=0.98;  % ������Ч��
eff_diff=0.98;% ������Ч��
eff_gear=0.98;

%% �����Ų���
K_PG1=2.1;      % ������1ϵ��
eff_PG1=0.98;    % ������1Ч��
fd_ratio=4.33; 
ratio_gear1=4.95;  
ratio_gear2=2; 

%% ����������

load 'engine.mat';
fuel_cal_value=42500; % kJ/kg  ȼ����ֵ
fuel_den=0.833;   %kg/L ȼ���ܶ� 

%% ���
% 68�����崮��
U_idle_x=[0,4,10,20,30,40,50,60,70,80,90,100]/100;
U_idle_y=[2.8797,3.3004,3.4709,3.5584,3.6219,3.6542,3.6935,3.7549,3.8439,3.9329,4.0367,4.1849]*145;
Q_QHmax=28*3600; %% ����28 Ah
R_soc = [30,40,50,60,70,80]/100;
R_Ch = [0.00189,0.00195,0.00207,0.00221,0.00231,0.00233]*145; % �������
R_Dis = [0.00276,0.00225,0.00197,0.00186,0.00185,0.00189]*145; % �ŵ�����

%% ���MG1
load MG1_data.mat;
MG1_Spd_mesh=xgi; MG1_Trq_mesh=ygi;
MG1_Eff_mesh=zgi; 
MG1_MaxPow=MG1_MaxSpd.*MG1_MaxTrq/9549;   
figure('name','MG1����'); 
plot(MG1_MaxSpd,MG1_MaxPow,'b-','LineWidth',2); 
xlabel('MG1ת�� rpm'); 
ylabel('MG1���� kW'); 
%% ���MG2
load MG2_data.mat;
MG2_Spd_mesh=xmi; MG2_Trq_mesh=ymi;
MG2_Eff_mesh=zmi; 
MG2_MaxPow=MG2_MaxSpd.*MG2_MaxTrq/9549;   
figure('name','MG2����'); 
plot(MG2_MaxSpd,MG2_MaxPow,'b-','LineWidth',2); 
xlabel('MG2ת�� rpm'); 
ylabel('MG2���� kW'); 
%% ʱ�䲽��
dt=0.1; 

%% ����
cyc_grade=atan(0);
%% ���Ʋ���

PowEngL2=30;
%%
v_min_zhiqu=70; %ֱ������70
%% 35km/h
soc_initial=0.82;   
%% 25km/h
soc_ChargeTop=0.81; % SOC��ֵ����
soc_control=0.807;    % SOC����ֵ
PowEngL=30;          % ��������������
ChargePower=30;      % �������
v_min_regen=10; % �����ƶ��������� 
kmh=35;   % ��������
 



