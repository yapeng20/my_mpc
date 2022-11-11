clear;
close all; 
clc;
% v = 0:147;
% v = v';
% T_hev = 1;
%% 整车基本参数
veh_gravity=9.8;    % m/s^2 重力加速度 
veh_air_density=1.2258; % kg/m^3 空气密度
veh_CD=0.61; % 空气阻力系数 
veh_FA=3.6; % (m^2) 
veh_mass=6000; %kg 最大总质量；整备质量6000 kg
wh_radius=0.376;    % (m), 7.00 R16 
wh_0st_rrc=423.1; % (--), 滚阻系数
wh_1st_rrc=2.324; % (--), 滚阻系数
wh_2st_rrc=0.17285; % (--), 滚阻系数



load waitexing.mat;
F = T_hev ./ wh_radius; 
Fr=wh_0st_rrc + wh_1st_rrc * (v ) + wh_2st_rrc * (v ).^2;
% Fw=0.5*veh_air_density*veh_CD*veh_FA.*(v/3.6).^2;
a = (F -Fr) / veh_mass;



% T = interp1(v,T_hev,1);