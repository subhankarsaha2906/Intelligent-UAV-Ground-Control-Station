clc;
clear;
close all;

%% ===============================================================
% UAV GROUND CONTROL STATION (VERSION 6)
% Author : Subhankar Saha
% ================================================================

disp('========================================================');
disp('        UAV GROUND CONTROL STATION - VERSION 6');
disp('========================================================');

%% ===============================================================
% SIMULATION SETTINGS
% ===============================================================

dt = 1;
MissionTime = 600;

time = 0:dt:MissionTime;

N = length(time);

%% ===============================================================
% MISSION PHASES
% ===============================================================

MissionPhase = strings(N,1);

for k = 1:N

    t = time(k);

    if t < 60

        MissionPhase(k)="TAKEOFF";

    elseif t < 120

        MissionPhase(k)="CLIMB";

    elseif t < 350

        MissionPhase(k)="CRUISE";

    elseif t < 450

        MissionPhase(k)="SURVEY";

    elseif t < 540

        MissionPhase(k)="RETURN";

    else

        MissionPhase(k)="LANDING";

    end

end

%% ===============================================================
% ALTITUDE MODEL
% ===============================================================

Altitude = zeros(1,N);

for k = 1:N

    t = time(k);

    if t<=80

        Altitude(k)=150*t/80;

    elseif t<=450

        Altitude(k)=150;

    elseif t<=520

        Altitude(k)=150-(150/70)*(t-450);

    else

        Altitude(k)=0;

    end

end
%% ===============================================================
% SPEED MODEL
% ===============================================================

Speed = 12 + 2*sin(0.03*time);

Speed(time>540)=5;

Speed(time>585)=0;

%% ===============================================================
% BATTERY MODEL
% ===============================================================

Battery = linspace(100,48,N);

BatteryHealth = 99.2;

BatteryVoltage = 16.8-(16.8-14.4)*(time/MissionTime);

BatteryCurrent = 5+0.6*sin(0.08*time);

BatteryPower = BatteryVoltage.*BatteryCurrent;

%% ===============================================================
% TEMPERATURE MODEL
% ===============================================================

ESC_Temperature = 30+12*(time/MissionTime);

MotorTemperature = 28+10*(time/MissionTime);

BatteryTemperature = 27+8*(time/MissionTime);

%% ===============================================================
% MOTOR MODEL
% ===============================================================

MotorRPM = 4800+350*sin(0.05*time);

%% ===============================================================
% WIND MODEL
% ===============================================================

WindSpeed = 5+2*sin(0.015*time);

WindDirection = mod(180+0.30*time,360);

%% ===============================================================
% GPS MODEL
% ===============================================================

Latitude = 22.5726+0.00001*time;

Longitude = 88.3640+0.000012*time;

Distance=zeros(1,N);

for k=2:N

    Distance(k)=Distance(k-1)+Speed(k)/1000;

end

%% ===============================================================
% AIRCRAFT ATTITUDE
% ===============================================================

Roll = 12*sin(0.025*time);

Pitch = 8*cos(0.020*time);

Yaw = mod(0.7*time,360);

%% ===============================================================
% COMMUNICATION MODEL
% ===============================================================

Signal = 98-18*(time/MissionTime);

SatelliteCount = 18-floor(time/120);

HDOP = 0.70+0.002*time;

%% ===============================================================
% POWER SYSTEM
% ===============================================================

PowerConsumed = cumtrapz(time,BatteryPower)/3600;

EnergyRemaining = Battery(end)/100*120;

%% ===============================================================
% FLIGHT STATUS
% ===============================================================

fprintf('\n');
disp('Simulation Started Successfully');
disp('Flight Controller Online');
disp('Navigation System Online');
disp('Battery Management System Online');
disp('Telemetry Link Established');

fprintf('\nMission Duration : %.0f sec\n',MissionTime);
fprintf('Simulation Points : %d\n',N);

disp(' ');
%% ===============================================================
% UAV SAFETY MONITOR
% ===============================================================

disp('========================================================');
disp('               UAV SAFETY MONITOR');
disp('========================================================');

LowBattery = Battery(end) < 30;
HighTemperature = ESC_Temperature(end) > 45;
WeakSignal = Signal(end) < 60;
HighWind = WindSpeed(end) > 10;
PoorGPS = SatelliteCount(end) < 8;

fprintf('Low Battery        : %s\n',string(LowBattery));
fprintf('High Temperature   : %s\n',string(HighTemperature));
fprintf('Weak Signal        : %s\n',string(WeakSignal));
fprintf('High Wind          : %s\n',string(HighWind));
fprintf('Poor GPS           : %s\n',string(PoorGPS));

disp(' ');

%% ===============================================================
% BATTERY ANALYTICS
% ===============================================================

BatteryConsumptionRate = ...
(Battery(1)-Battery(end))/MissionTime;

RemainingBattery = Battery(end);

RemainingFlightTime = ...
RemainingBattery/BatteryConsumptionRate;

RemainingRange = ...
mean(Speed)*RemainingFlightTime/1000;

BatterySOH = BatteryHealth;

BatteryEfficiency = ...
(BatterySOH/100)*(RemainingBattery/100)*100;

disp('========================================================');
disp('BATTERY ANALYTICS');
disp('========================================================');

fprintf('Battery SOC            : %.2f %%\n',RemainingBattery);
fprintf('Battery SOH            : %.2f %%\n',BatterySOH);
fprintf('Consumption Rate       : %.3f %%/sec\n',BatteryConsumptionRate);
fprintf('Remaining Flight Time  : %.1f sec\n',RemainingFlightTime);
fprintf('Remaining Range        : %.2f km\n',RemainingRange);
fprintf('Battery Efficiency     : %.2f %%\n',BatteryEfficiency);

disp(' ');

%% ===============================================================
% GPS HEALTH
% ===============================================================

if SatelliteCount(end)>=15

    GPSStatus="EXCELLENT";

elseif SatelliteCount(end)>=10

    GPSStatus="GOOD";

elseif SatelliteCount(end)>=6

    GPSStatus="FAIR";

else

    GPSStatus="POOR";

end

disp('========================================================');
disp('GPS HEALTH');
disp('========================================================');

fprintf('GPS Status         : %s\n',GPSStatus);
fprintf('Satellite Count    : %d\n',SatelliteCount(end));
fprintf('HDOP               : %.2f\n',HDOP(end));

disp(' ');

%% ===============================================================
% FAULT DETECTION ENGINE
% ===============================================================

FaultList = {};

if LowBattery
    FaultList{end+1}='LOW BATTERY';
end

if HighTemperature
    FaultList{end+1}='HIGH TEMPERATURE';
end

if WeakSignal
    FaultList{end+1}='WEAK SIGNAL';
end

if HighWind
    FaultList{end+1}='HIGH WIND';
end

if PoorGPS
    FaultList{end+1}='GPS FAILURE';
end

disp('========================================================');
disp('FAULT DIAGNOSTICS');
disp('========================================================');

if isempty(FaultList)

    disp('No Active Faults');

else

    disp(FaultList');

end

disp(' ');

%% ===============================================================
% RETURN TO HOME
% ===============================================================

RTH = false;

Reason = "";

if LowBattery

    RTH = true;
    Reason = "LOW BATTERY";

elseif WeakSignal

    RTH = true;
    Reason = "WEAK SIGNAL";

elseif HighTemperature

    RTH = true;
    Reason = "HIGH TEMPERATURE";

elseif HighWind

    RTH = true;
    Reason = "HIGH WIND";

elseif PoorGPS

    RTH = true;
    Reason = "GPS FAILURE";

end

disp('========================================================');
disp('RETURN TO HOME');
disp('========================================================');

if RTH

    MissionStatus = "RETURN HOME";

else

    MissionStatus = "CONTINUE";

end

fprintf('Mission Status : %s\n',MissionStatus);

if RTH
    fprintf('Reason         : %s\n',Reason);
end

disp(' ');

%% ===============================================================
% EMERGENCY LANDING
% ===============================================================

EmergencyLanding = false;

EmergencyReason = "";

if Battery(end)<15

    EmergencyLanding = true;
    EmergencyReason = "CRITICAL BATTERY";

elseif ESC_Temperature(end)>55

    EmergencyLanding = true;
    EmergencyReason = "ESC OVERHEATING";

elseif Signal(end)<20

    EmergencyLanding = true;
    EmergencyReason = "COMMUNICATION FAILURE";

end

disp('========================================================');
disp('EMERGENCY LANDING');
disp('========================================================');

if EmergencyLanding

    fprintf('Emergency Landing : YES\n');
    fprintf('Reason            : %s\n',EmergencyReason);

else

    fprintf('Emergency Landing : NOT REQUIRED\n');

end

disp(' ');

%% ===============================================================
% MISSION SCORE
% ===============================================================

BatteryScore = BatterySOH;

SignalScore = Signal(end);

GPSScore = SatelliteCount(end)*5;

TemperatureScore = max(0,100-ESC_Temperature(end));

MissionScore = mean([BatteryScore ...
                     SignalScore ...
                     GPSScore ...
                     TemperatureScore]);

disp('========================================================');
disp('MISSION ANALYTICS');
disp('========================================================');

fprintf('Battery Score      : %.2f\n',BatteryScore);
fprintf('Signal Score       : %.2f\n',SignalScore);
fprintf('GPS Score          : %.2f\n',GPSScore);
fprintf('Temperature Score  : %.2f\n',TemperatureScore);

fprintf('\nOverall Mission Score : %.2f /100\n',MissionScore);

disp(' ');

%% ===============================================================
% FLIGHT HEALTH INDEX
% ===============================================================

FlightHealth = mean([BatterySOH ...
                     Signal(end) ...
                     100-HDOP(end)]);

fprintf('Flight Health Index : %.2f %%\n',FlightHealth);

if FlightHealth>90

    HealthStatus="EXCELLENT";

elseif FlightHealth>80

    HealthStatus="GOOD";

elseif FlightHealth>65

    HealthStatus="FAIR";

else

    HealthStatus="POOR";

end

fprintf('Overall Health      : %s\n',HealthStatus);

disp(' ');
%% ===============================================================
% UAV GROUND CONTROL STATION DASHBOARD
% ===============================================================

DashboardFig = figure(...
    'Name','UAV Ground Control Station V6',...
    'NumberTitle','off',...
    'Color',[0.12 0.12 0.12],...
    'Position',[20 20 1800 950]);

tiledlayout(3,5,'TileSpacing','compact','Padding','compact');
nexttile

plot(time,Altitude,'b','LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude')
set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,Speed,'m','LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Speed (m/s)')
title('Speed')
set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,Battery,'g','LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('%')
title('Battery SOC')
set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,BatteryVoltage,'c','LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Battery Voltage')
set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,BatteryCurrent,'y','LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Current (A)')
title('Battery Current')
set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,BatteryPower,...
    'Color',[1 0.5 0],...
    'LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('Power (W)')
title('Power Consumption')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,ESC_Temperature,'r','LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('Temp (°C)')
title('ESC Temperature')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,WindSpeed,...
    'Color',[0 0.8 1],...
    'LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('Wind (m/s)')
title('Wind Speed')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,Signal,'w','LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('Signal (%)')
title('Signal Strength')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,MotorRPM,...
    'Color',[1 0 1],...
    'LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('RPM')
title('Motor RPM')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,Roll,'b','LineWidth',2)
hold on
plot(time,Pitch,'r','LineWidth',2)

legend('Roll','Pitch',...
    'Location','southwest')

grid on
xlabel('Time (s)')
ylabel('Degrees')
title('Aircraft Attitude')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(time,Yaw,'g','LineWidth',2)

grid on
xlabel('Time (s)')
ylabel('Degrees')
title('Heading (Yaw)')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

plot(Longitude,Latitude,'c','LineWidth',2)
hold on

scatter(Longitude(1),Latitude(1),80,...
    'g','filled')

scatter(Longitude(end),Latitude(end),80,...
    'r','filled')

grid on
xlabel('Longitude')
ylabel('Latitude')
title('GPS Flight Path')

legend('Flight Path',...
    'Start',...
    'End')

set(gca,'Color',[0.18 0.18 0.18],...
    'XColor','w','YColor','w')
nexttile

axis off

text(0,1,'MISSION SUMMARY',...
    'Color','w',...
    'FontWeight','bold',...
    'FontSize',14)

text(0,0.88,...
    sprintf('Mission Time : %.0f sec',MissionTime),...
    'Color','g')

text(0,0.76,...
    sprintf('Distance : %.2f km',Distance(end)),...
    'Color','c')

text(0,0.64,...
    sprintf('Battery : %.1f %%',Battery(end)),...
    'Color','y')

text(0,0.52,...
    sprintf('Mission Score : %.1f',MissionScore),...
    'Color','m')

text(0,0.40,...
    sprintf('GPS Status : %s',GPSStatus),...
    'Color','w')

text(0,0.28,...
    sprintf('Health : %s',HealthStatus),...
    'Color','w')

if RTH

    text(0,0.12,...
        'STATUS : RETURN HOME',...
        'Color','r',...
        'FontWeight','bold');

else

    text(0,0.12,...
        'STATUS : CONTINUE',...
        'Color','g',...
        'FontWeight','bold');

end
%% ===============================================================
% SAVE DASHBOARD
% ===============================================================

saveas(DashboardFig,'UAV_GCS_V6_Dashboard.png');

disp(' ');
disp('Dashboard saved successfully.');
%% ===============================================================
% AI MISSION RISK PREDICTOR
% ===============================================================

disp(' ');
disp('========================================================');
disp('AI MISSION RISK ANALYSIS');
disp('========================================================');

RiskScore = 0;

if Battery(end) < 50
    RiskScore = RiskScore + 25;
end

if Signal(end) < 75
    RiskScore = RiskScore + 20;
end

if ESC_Temperature(end) > 40
    RiskScore = RiskScore + 20;
end

if WindSpeed(end) > 8
    RiskScore = RiskScore + 15;
end

if SatelliteCount(end) < 10
    RiskScore = RiskScore + 20;
end

if RiskScore < 25
    MissionRisk = "LOW";
elseif RiskScore < 55
    MissionRisk = "MEDIUM";
else
    MissionRisk = "HIGH";
end

fprintf('Risk Score      : %.0f /100\n',RiskScore);
fprintf('Mission Risk    : %s\n',MissionRisk);

disp(' ');
%% ===============================================================
% BATTERY REMAINING USEFUL LIFE
% ===============================================================

BatteryCyclesRemaining = BatteryHealth * 12;

BatteryLifeDays = BatteryCyclesRemaining / 1.5;

disp('========================================================');
disp('BATTERY LIFE PREDICTION');
disp('========================================================');

fprintf('Remaining Cycles : %.0f\n',BatteryCyclesRemaining);
fprintf('Estimated Life   : %.0f Days\n',BatteryLifeDays);

disp(' ');
%% ===============================================================
% PREDICTIVE MAINTENANCE
% ===============================================================

disp('========================================================');
disp('PREDICTIVE MAINTENANCE');
disp('========================================================');

if BatteryHealth > 95

    BatteryMaintenance = "NOT REQUIRED";

elseif BatteryHealth > 90

    BatteryMaintenance = "INSPECT";

else

    BatteryMaintenance = "REPLACE";

end

if ESC_Temperature(end) > 42

    MotorMaintenance = "INSPECT MOTOR";

else

    MotorMaintenance = "NORMAL";

end

fprintf('Battery : %s\n',BatteryMaintenance);
fprintf('Motor   : %s\n',MotorMaintenance);

disp(' ');
%% ===============================================================
% FLIGHT EFFICIENCY
% ===============================================================

AverageSpeed = mean(Speed);

EnergyUsed = Battery(1)-Battery(end);

Efficiency = Distance(end)/EnergyUsed;

disp('========================================================');
disp('FLIGHT PERFORMANCE');
disp('========================================================');

fprintf('Average Speed : %.2f m/s\n',AverageSpeed);
fprintf('Energy Used   : %.2f %%\n',EnergyUsed);
fprintf('Efficiency    : %.3f km per Battery %%\n',Efficiency);

disp(' ');
%% ===============================================================
% AUTONOMOUS DECISION ENGINE
% ===============================================================

disp('========================================================');
disp('AUTONOMOUS DECISION ENGINE');
disp('========================================================');

if EmergencyLanding

    Recommendation = "LAND IMMEDIATELY";

elseif RTH

    Recommendation = "RETURN TO HOME";

elseif MissionRisk=="HIGH"

    Recommendation = "ABORT MISSION";

elseif MissionRisk=="MEDIUM"

    Recommendation = "PROCEED WITH CAUTION";

else

    Recommendation = "CONTINUE MISSION";

end

fprintf('Recommended Action : %s\n',Recommendation);

disp(' ');
%% ===============================================================
% FINAL MISSION GRADE
% ===============================================================

OverallGrade = MissionScore - RiskScore/2;

if OverallGrade > 90

    Grade = 'A';

elseif OverallGrade > 80

    Grade = 'B';

elseif OverallGrade > 70

    Grade = 'C';

else

    Grade = 'D';

end

disp('========================================================');
disp('MISSION REPORT');
disp('========================================================');

fprintf('Mission Score : %.2f\n',MissionScore);
fprintf('Mission Risk  : %s\n',MissionRisk);
fprintf('Mission Grade : %c\n',Grade);

disp(' ');
%% ===============================================================
% SYSTEM HEALTH INDEX
% ===============================================================

SystemHealth = mean([...
    BatterySOH,...
    Signal(end),...
    100-HDOP(end),...
    100-ESC_Temperature(end)]);

fprintf('Overall System Health : %.2f %%\n',SystemHealth);

if SystemHealth>90

    SystemState="EXCELLENT";

elseif SystemHealth>80

    SystemState="GOOD";

elseif SystemHealth>70

    SystemState="FAIR";

else

    SystemState="POOR";

end

fprintf('System Status : %s\n',SystemState);

disp(' ');
%% ===============================================================
% FINAL SUMMARY
% ===============================================================

disp('========================================================');
disp('FINAL FLIGHT SUMMARY');
disp('========================================================');

fprintf('Mission Duration   : %.0f sec\n',MissionTime);
fprintf('Distance Travelled : %.2f km\n',Distance(end));
fprintf('Battery Remaining  : %.2f %%\n',Battery(end));
fprintf('Mission Score      : %.2f\n',MissionScore);
fprintf('Mission Risk       : %s\n',MissionRisk);
fprintf('Recommendation     : %s\n',Recommendation);
fprintf('System Status      : %s\n',SystemState);

disp('========================================================');
%% ===============================================================
% CREATE PROJECT FOLDERS
% ===============================================================

if ~exist('reports','dir')
    mkdir('reports');
end

if ~exist('images','dir')
    mkdir('images');
end

disp('Project folders verified.');
%% ===============================================================
% EXPORT TELEMETRY DATA
% ===============================================================

TelemetryTable = table(...
    time',...
    Altitude',...
    Speed',...
    Battery',...
    BatteryVoltage',...
    BatteryCurrent',...
    BatteryPower',...
    ESC_Temperature',...
    MotorRPM',...
    WindSpeed',...
    Signal',...
    Latitude',...
    Longitude',...
    Roll',...
    Pitch',...
    Yaw',...
    Distance',...
    'VariableNames',...
    {'Time_s',...
    'Altitude_m',...
    'Speed_mps',...
    'Battery_SOC',...
    'Voltage_V',...
    'Current_A',...
    'Power_W',...
    'Temperature_C',...
    'MotorRPM',...
    'WindSpeed_mps',...
    'Signal',...
    'Latitude',...
    'Longitude',...
    'Roll',...
    'Pitch',...
    'Yaw',...
    'Distance_km'});

writetable(TelemetryTable,'reports/Flight_Report.csv');

disp('Flight_Report.csv created successfully.');
%% ===============================================================
% SAVE DASHBOARD
% ===============================================================
if ~exist('images','dir')
    mkdir('images');
end
disp(DashboardFig)
isvalid(DashboardFig)

if ~exist('images','dir')

mkdir('images');

end

mkdir('images');

cd('images');

saveas(DashboardFig,'UAV_GCS_Dashboard.png');

cd ..

movefile('UAV_GCS_Dashboard.png','images/UAV_GCS_Dashboard.png');

disp('Dashboard image saved.');
%% ===============================================================
% GENERATE REPORT
% ===============================================================

fid = fopen('reports/Mission_Report.txt','w');

fprintf(fid,'=============================================\n');
fprintf(fid,'UAV GROUND CONTROL STATION VERSION 6\n');
fprintf(fid,'=============================================\n\n');

fprintf(fid,'Mission Duration        : %.0f sec\n',MissionTime);
fprintf(fid,'Distance Travelled      : %.2f km\n',Distance(end));
fprintf(fid,'Average Speed           : %.2f m/s\n',AverageSpeed);

fprintf(fid,'Battery Remaining       : %.2f %%\n',Battery(end));
fprintf(fid,'Battery Health          : %.2f %%\n',BatteryHealth);

fprintf(fid,'Mission Score           : %.2f\n',MissionScore);
fprintf(fid,'Mission Risk            : %s\n',MissionRisk);
fprintf(fid,'Mission Grade           : %c\n',Grade);

fprintf(fid,'System Health           : %.2f %%\n',SystemHealth);

fprintf(fid,'Recommendation          : %s\n',Recommendation);

fprintf(fid,'GPS Status              : %s\n',GPSStatus);

fprintf(fid,'\n');

if RTH
    fprintf(fid,'Return To Home : YES\n');
else
    fprintf(fid,'Return To Home : NO\n');
end

if EmergencyLanding
    fprintf(fid,'Emergency Landing : YES\n');
else
    fprintf(fid,'Emergency Landing : NO\n');
end

fprintf(fid,'\nGenerated automatically using MATLAB\n');

fclose(fid);

disp('Mission report generated.');
%% ===============================================================
% FLIGHT STATISTICS
% ===============================================================

MaximumAltitude = max(Altitude);

MaximumSpeed = max(Speed);

MaximumWind = max(WindSpeed);

MaximumTemperature = max(ESC_Temperature);

AveragePower = mean(BatteryPower);

fprintf('\n');
disp('================ FLIGHT STATISTICS ==================');

fprintf('Maximum Altitude     : %.2f m\n',MaximumAltitude);
fprintf('Maximum Speed        : %.2f m/s\n',MaximumSpeed);
fprintf('Maximum Wind Speed   : %.2f m/s\n',MaximumWind);
fprintf('Maximum Temperature  : %.2f C\n',MaximumTemperature);
fprintf('Average Power        : %.2f W\n',AveragePower);

disp('=====================================================');
%% ===============================================================
% MISSION SUCCESS
% ===============================================================

MissionSuccess = ...
    (~EmergencyLanding) && ...
    (~RTH) && ...
    (Battery(end)>20);

fprintf('\n');

if MissionSuccess

    disp('MISSION STATUS : SUCCESS');

else

    disp('MISSION STATUS : ATTENTION REQUIRED');

end
%% ===============================================================
% END OF PROJECT
% ===============================================================

disp(' ');
disp('=====================================================');
disp('UAV GROUND CONTROL STATION VERSION 6');
disp('Project Execution Completed Successfully');
disp('=====================================================');

disp('Generated Files:');
disp(' ');
disp('images/UAV_GCS_Dashboard.png');
disp('reports/Flight_Report.csv');
disp('reports/Mission_Report.txt');

disp(' ');
disp('Thank you for using UAV Ground Control Station V6');
disp('=====================================================');