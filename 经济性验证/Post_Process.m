close all;
%%
% 发动机工作点 

open 万有特性.fig;
hold on
id1=find(sim_Mode==2);
plot(sim_Engine_Spd_out(id1),sim_Engine_Trq_out(id1),'r.','LineWidth',3);
id2=find(sim_Mode==5);
plot(sim_Engine_Spd_out(id2),sim_Engine_Trq_out(id2),'b.','LineWidth',3);
xlabel('转速（rpm）','FontSize',14);
ylabel('转矩（Nm)','FontSize',14);
title('发动机工作点分布','FontSize',14);
ylim([0,384]);
xlim([1000,3500]);
saveas(gcf,'发动机工作点分布（二维）','fig');   

%% 电机工作点MG1
open MG1效率map.fig
id1=find((sim_Mode==1|sim_Mode==2|sim_Mode==3)&(gear_num==1));
id2=find((sim_Mode==1|sim_Mode==2|sim_Mode==3)&(gear_num==2));
plot(sim_MG1_Spd_out(id1),sim_MG1_Trq_out(id1),'r.'); hold on;
plot(sim_MG1_Spd_out(id2),sim_MG1_Trq_out(id2),'g*'); 
grid;
xlabel('转速（rpm)','FontSize',14);
ylabel('转矩（Nm)','FontSize',14);
title('MG1工作点分布','FontSize',14);
saveas(gcf,'MG1工作点分布（二维）','fig');   
%% 电机工作点MG2
open MG2效率map.fig;
id1=find((sim_Mode==1|sim_Mode==2|sim_Mode==3)&(gear_num==1));
id2=find((sim_Mode==1|sim_Mode==2|sim_Mode==3)&(gear_num==2));
plot(sim_MG2_Spd_out(id1),sim_MG2_Trq_out(id1),'r.'); hold on;
plot(sim_MG2_Spd_out(id2),sim_MG2_Trq_out(id2),'g*'); 
grid;
xlabel('转速（rpm)','FontSize',14);
ylabel('转矩（Nm)','FontSize',14);
title('MG2工作点分布','FontSize',14);
% legend('一挡','二挡');
saveas(gcf,'MG2工作点分布（二维）','fig');   
%% 发动机输出能量
% 发动机输出总能量
Energy_engine=trapz(sim_time,sim_Engine_power); % kJ
%计算发动机纯油耗
Engine_fuel_calculated=sim_fuelall_L(end); % L
Energy_fuel=Engine_fuel_calculated*fuel_den*fuel_cal_value; % kJ
id_km1=find(clock==900); id_km2=find(clock==1368);  id_km3=find(clock==1800);
Engine_fuel_900=sim_fuelall_L(id_km1);  Distance_900=sim_Distance(id_km1);
Engine_fuel_468=sim_fuelall_L(id_km2)-sim_fuelall_L(id_km1);  Distance_468=sim_Distance(id_km2)-sim_Distance(id_km1);
Engine_fuel_432=sim_fuelall_L(id_km3)-sim_fuelall_L(id_km2);  Distance_432=sim_Distance(id_km3)-sim_Distance(id_km2);
Engine_fuel_100km=100*(0.1*Engine_fuel_900/Distance_900+0.6*Engine_fuel_468/Distance_468+0.3*Engine_fuel_432/Distance_432);

%发动机平均燃油消耗率
ICE_gkwh_mean=Engine_fuel_calculated*fuel_den*1000/Energy_engine*3600; % g/kwh
id=find(sim_Mode==2|sim_Mode==5);
Engine_gkwh_mean=sum(sim_Engine_gkwh(id))/length(id);
%发动机平均热效率
Engine_efficiency_mean1=3600*1000/ICE_gkwh_mean/44000;
Engine_efficiency_mean2=sum(sim_Engine_heat_efficiency(id))/length(id);
%% 电池输出能量
%计算电池电耗
soc_trapz=linspace(sim_SOC(1),sim_SOC(end),20); % soc
    Ah_trapz=Q_QHmax/3600*soc_trapz; % Ah
    uidle_trapz=interp1(U_idle_x,U_idle_y,soc_trapz); % U
    Ah_trapz=Ah_trapz*(sim_SOC(end)>=sim_SOC(1))+fliplr(Ah_trapz)*(sim_SOC(end)<sim_SOC(1));
    uidle_trapz=uidle_trapz*(sim_SOC(end)>=sim_SOC(1))+fliplr(uidle_trapz)*(sim_SOC(end)<sim_SOC(1));
    Energy_battery=trapz(Ah_trapz*3600/1000,uidle_trapz);
    Energy_battery=Energy_battery*(sim_SOC(end)<sim_SOC(1))-Energy_battery*(sim_SOC(end)>=sim_SOC(1));
%计算电池平均驱动充电效率
    Eff_battery_charge=sim_battery_efficiency.*(sim_battery_power<0).*(sim_Mode==2);
    id=find(Eff_battery_charge>0);
    Eff_battery_charge_mean=sum(Eff_battery_charge(id))/length(id);
    clear id;
%计算电池平均放电效率
    Eff_battery_discharge=sim_battery_efficiency.*(sim_battery_power>0);
    id=find(Eff_battery_discharge>0);
    Eff_battery_discharge_mean=sum(Eff_battery_discharge(id))/length(id);
    clear id;
%计算电池电耗折算的油耗
    Energy_battery_input=Energy_battery/Eff_battery_charge_mean*(sim_SOC(end)>sim_SOC(1))+Energy_battery/Eff_battery_discharge_mean*(sim_SOC(end)<sim_SOC(1));%KW
    Battery_fuel=Energy_battery_input*Engine_gkwh_mean/1000/3600/fuel_den/sim_Distance(end)*100;%L/100km
%%
%驱动总能量
Energy_drive=trapz(sim_time,sim_Power_ground.*(sim_Power_ground>0));
%再生制动回收能量
Energy_RGB=trapz(sim_time,(sim_MG2_ele_power.*(sim_Mode==3)));
%计算综合油耗
    fuel_sythesis_100kmh=Engine_fuel_100km+Battery_fuel;
%计算系统平均效率
   Energy_Battery_Dis1=sim_battery_power.*(sim_battery_power>0);
   Energy_Battery_Dis=sum(Energy_Battery_Dis1*dt)/Eff_battery_discharge_mean;
   Energy_Battery_Ch_drive1=sim_battery_power.*(sim_battery_power<0).*(sim_Mode==2);
   Energy_Battery_Ch_drive=sum(Energy_Battery_Ch_drive1*dt)*Eff_battery_charge_mean;
   Eff_mean_sythesis=Energy_drive/(Energy_fuel+Energy_Battery_Dis+Energy_Battery_Ch_drive);
   %统计各模式时间占比
    percent_EV=sum(sim_Mode==1)/length(sim_Mode)*100;
    percent_EVT=sum(sim_Mode==2)/length(sim_Mode)*100;
    percent_RGB=sum(sim_Mode==3)/length(sim_Mode)*100;
    percent_MB=sum(sim_Mode==4)/length(sim_Mode)*100;
     percent_fz=sum(sim_Mode==5)/length(sim_Mode)*100;
%% 计算电机平均效率
% 计算MG1平均发电效率
id1=find(sim_MG1_ele_power<0);
if length(id1>0)
    MG1_generate_input_energy=sum(sim_MG1_mech_power(id1)*dt);
    MG1_generate_output_energy=sum(sim_MG1_ele_power(id1)*dt);
    MG1_mean_generate_eff=MG1_generate_output_energy/MG1_generate_input_energy;
else
    MG1_generate_input_energy=0;
    MG1_generate_output_energy=0;
    MG1_mean_generate_eff=0;
end
% 计算MG1平均电动效率
id2=find(sim_MG1_ele_power>0);
if length(id2>0)
    MG1_drive_input_energy=sum(sim_MG1_ele_power(id2)*dt);
    MG1_drive_output_energy=sum(sim_MG1_mech_power(id2)*dt);
    MG1_mean_drive_eff=MG1_drive_output_energy/MG1_drive_input_energy;
else
    MG1_drive_input_energy=0;
    MG1_drive_output_energy=0;
    MG1_mean_drive_eff=0;
end
% 计算MG1平均效率
MG1_mean_eff=(abs(MG1_generate_output_energy)+MG1_drive_output_energy)/(abs(MG1_generate_input_energy)+MG1_drive_input_energy);

% 计算MG2平均发电效率
id3=find(sim_MG2_ele_power<0);
if length(id3>0)
    MG2_generate_input_energy=sum(sim_MG2_mech_power(id3)*dt);
    MG2_generate_output_energy=sum(sim_MG2_ele_power(id3)*dt);
    MG2_mean_generate_eff=MG2_generate_output_energy/MG2_generate_input_energy;
else
    MG2_generate_input_energy=0;
    MG2_generate_output_energy=0;
    MG2_mean_generate_eff=0;
end
% 计算MG2平均电动效率
id4=find(sim_MG2_ele_power>0);
if length(id4>0)
    MG2_drive_input_energy=sum(sim_MG2_ele_power(id4)*dt);
    MG2_drive_output_energy=sum(sim_MG2_mech_power(id4)*dt);
    MG2_mean_drive_eff=MG2_drive_output_energy/MG2_drive_input_energy;
else
    MG2_drive_input_energy=0;
    MG2_drive_output_energy=0;
    MG2_mean_drive_eff=0;
end
% 计算MG2平均效率
MG2_mean_eff=(abs(MG2_generate_output_energy)+MG2_drive_output_energy)/(abs(MG2_generate_input_energy)+MG2_drive_input_energy);
%% 发动机三维工作点 
    ice_spd_segament1=1485;ice_spd_segament2=1575;
    ice_trq_segament1=291.8;ice_trq_segament2=338.8;
    id=find(sim_Mode==2);
    ICE_spd_distribution=sim_Engine_Spd_out(id);
    ICE_trq_distribution=sim_Engine_Trq_out(id);
    ICE_segament1_1=find(ICE_spd_distribution<=ice_spd_segament1&ICE_trq_distribution<=ice_trq_segament1);
    ICE_segament1_2=find(ICE_spd_distribution<=ice_spd_segament1&ICE_trq_distribution>ice_trq_segament1&ICE_trq_distribution<=ice_trq_segament2);
    ICE_segament1_3=find(ICE_spd_distribution<=ice_spd_segament1&ICE_trq_distribution>ice_trq_segament2);
    ICE_segament2_1=find(ICE_spd_distribution>ice_spd_segament1&ICE_spd_distribution<=ice_spd_segament2&ICE_trq_distribution<=ice_trq_segament1);
    ICE_segament2_2=find(ICE_spd_distribution>ice_spd_segament1&ICE_spd_distribution<=ice_spd_segament2&ICE_trq_distribution>ice_trq_segament1&ICE_trq_distribution<=ice_trq_segament2);
    ICE_segament2_3=find(ICE_spd_distribution>ice_spd_segament1&ICE_spd_distribution<=ice_spd_segament2&ICE_trq_distribution>ice_trq_segament2);
    ICE_segament3_1=find(ICE_spd_distribution>ice_spd_segament2&ICE_trq_distribution<=ice_trq_segament1);
    ICE_segament3_2=find(ICE_spd_distribution>ice_spd_segament2&ICE_trq_distribution>ice_trq_segament1&ICE_trq_distribution<=ice_trq_segament2);
    ICE_segament3_3=find(ICE_spd_distribution>ice_spd_segament2&ICE_trq_distribution>ice_trq_segament2);
    
    percent_ICE_segament1_1=length(ICE_segament1_1)/length(ICE_spd_distribution)*100;
    percent_ICE_segament1_2=length(ICE_segament1_2)/length(ICE_spd_distribution)*100;
    percent_ICE_segament1_3=length(ICE_segament1_3)/length(ICE_spd_distribution)*100;
    percent_ICE_segament2_1=length(ICE_segament2_1)/length(ICE_spd_distribution)*100;
    percent_ICE_segament2_2=length(ICE_segament2_2)/length(ICE_spd_distribution)*100;
    percent_ICE_segament2_3=length(ICE_segament2_3)/length(ICE_spd_distribution)*100;
    percent_ICE_segament3_1=length(ICE_segament3_1)/length(ICE_spd_distribution)*100;
    percent_ICE_segament3_2=length(ICE_segament3_2)/length(ICE_spd_distribution)*100;
    percent_ICE_segament3_3=length(ICE_segament3_3)/length(ICE_spd_distribution)*100;
    percent_ICE_segament=[percent_ICE_segament1_1,percent_ICE_segament2_1,percent_ICE_segament3_1
                          percent_ICE_segament1_2,percent_ICE_segament2_2,percent_ICE_segament3_2
                          percent_ICE_segament1_3,percent_ICE_segament2_3,percent_ICE_segament3_3];
figure('name','发动机工作点分布');
bar3(percent_ICE_segament);
xlabel('发动机转速(rpm)','FontSize',18);ylabel('发动机扭矩（Nm）','FontSize',18);zlabel('占比（%）','FontSize',18);
title('发动机工作点分布','FontSize',18);
set(findobj( get(gca,'Children'),'LineWidth',0.5),'LineWidth',2);
%set(gcf,'Color','w');
set(gca,'FontSize',15);
set(gcf,'unit','centimeters','position',[15 5 20 15]);
set(gca,'XTickLabel',{'1000-1485','1485-1575','1575-6000'}) ;
set(gca,'YTickLabel',{'0-291.8','291.8-338.8','338.8-381'}) ;
saveas(gcf,'发动机工作点分布（三维）','fig');


%% 损耗分析
% EV模式下系统损耗
id=find(sim_Mode==1);
P_loss_trans_EV=sum(sim_battery_power(id).*(sim_battery_power(id)>0).*(sim_battery_efficiency(id)>0)./sim_battery_efficiency(id)-sim_Power_ground(id))/length(id);
% EVT模式下的系统损耗
id=find(sim_Mode==2);
P_loss_trans_EVT=sum(sim_Engine_power(id)+sim_battery_power(id).*(sim_battery_power(id)>0)./sim_battery_efficiency(id)+sim_battery_power(id).*sim_battery_efficiency(id).*(sim_battery_power(id)<0)-sim_Power_ground(id))/length(id);
P_loss_eng=sum(sim_Engine_power(id).*(sim_Engine_gkwh(id)*42500/3600/1000-1))/length(id);
% RGB模式下的系统回收
id=find(sim_Mode==3);
P_rgb=0;
%P_rgb=sum(sim_battery_power(id).*sim_battery_efficiency(id))/length(id);
%sum(sim_battery_power(id).*sim_battery_efficiency(id))/length(id);
% MB模式下的系统损失
id=find(sim_Mode==4);
P_mb=-sum(sim_Power_ground(id))/length(id);

% 系统平均功率损失
P_loss_avg=P_loss_trans_EV*percent_EV/100+(P_loss_eng+P_loss_trans_EVT)*percent_EVT/100+P_rgb*percent_RGB/100+P_mb*percent_MB/100;


% 最终结果显示
baigongliyouhao(end) 
sim_SOC(1)
sim_SOC(end)

