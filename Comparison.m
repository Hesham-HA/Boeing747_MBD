% Created in May 2021 by CUFE AER-2021 Seniors:
% Mahmoud Ahmed Moustafa, Osama Mamdouh, Mohamed Ahmed Ali, Hesham Hassan &
% Ahmed Elrawy for our Autopilot Project, under the instruction of
% Dr. Osama Mohamady and Eng. Wessam Ahmed.

%% Clearing Command Window and Workspace and Closing all Figures

clc
clearvars
close all

%% Aircraft Data

Data_From_Excel = readmatrix('B-747 Flight Condition 5.xlsx','Range','B2:B61');
  Aircraft_Data = AircraftData(Data_From_Excel);

%% Control Actions

Control_actions = [Data_From_Excel(57); Data_From_Excel(58); Data_From_Excel(59); Data_From_Excel(60)];
Aircraft_Data.CA = [Data_From_Excel(57)*pi/180; Data_From_Excel(58)*pi/180; Data_From_Excel(59)*pi/180; Data_From_Excel(60)];
if Aircraft_Data.CA(1)==0 && Aircraft_Data.CA(2)==0 && Aircraft_Data.CA(3)==0 && Aircraft_Data.CA(4)==0
    error('No input detected.')
elseif Aircraft_Data.CA(2)==0 && Aircraft_Data.CA(3)==0 && Aircraft_Data.CA(4)==0
    input_index = 1;
elseif Aircraft_Data.CA(1)==0 && Aircraft_Data.CA(3)==0 && Aircraft_Data.CA(4)==0
    input_index = 2;
elseif Aircraft_Data.CA(1)==0 && Aircraft_Data.CA(2)==0 && Aircraft_Data.CA(4)==0
    input_index = 3;
elseif  Aircraft_Data.CA(1)==0 && Aircraft_Data.CA(2)==0 && Aircraft_Data.CA(3)==0
    input_index = 4;
else
    input_index = NaN;
end

%% Non-linear Airplane Equations of Motion (NLAEOM)

NL = NLAEOM_Model(Aircraft_Data);

%% Simulink Model
% Running Simulink model
 SimOut = sim('Airplane_NLSimulator.slx');

% Extracting data from the Simulink model
    SM.t = SimOut.SS_Simulink.Time;
    SM.u = SimOut.SS_Simulink.Data(:,7);
    SM.v = SimOut.SS_Simulink.Data(:,8);
    SM.w = SimOut.SS_Simulink.Data(:,9);
    SM.p = SimOut.SS_Simulink.Data(:,10);
    SM.q = SimOut.SS_Simulink.Data(:,11);
    SM.r = SimOut.SS_Simulink.Data(:,12);
  SM.phi = SimOut.SS_Simulink.Data(:,4);
SM.theta = SimOut.SS_Simulink.Data(:,5);
  SM.psi = SimOut.SS_Simulink.Data(:,6);
   SM.xE = SimOut.SS_Simulink.Data(:,1);
   SM.yE = SimOut.SS_Simulink.Data(:,2);
   SM.zE = SimOut.SS_Simulink.Data(:,3);
SM.alpha = SimOut.SS_Simulink.Data(:,13);
 SM.beta = SimOut.SS_Simulink.Data(:,14);

%% Plotting Commands

% Strings for figures
str1 = ["\Delta\delta_{a}","\Delta\delta_{e}","\Delta\delta_{r}","\Delta\delta_{th}"];
str2 = ["^{\circ}","^{\circ}","^{\circ}","\:lbs"];

% u_body comparison
figure
if input_index==1 ||input_index==2 || input_index==3 || input_index==4
    sgtitle(['Comparison between the results of our RBD Solver and the Simulink Model for $',convertStringsToChars(str1(1,input_index)),' = ',num2str(Control_actions(input_index)),convertStringsToChars(str2(1,input_index)),'$'],'interpreter','latex','FontSize',20)
elseif isnan(input_index)
    sgtitle('Comparison between the results of our RBD Solver and the Simulink Model','interpreter','latex','FontSize',20)
else
    error('Check the instructions for setting the value of the variable "input_index".')
end
subplot(5,3,1)
hold on
plot(SM.t,SM.u,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.u,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$u_{b}$ (m/sec)','interpreter','latex','FontSize',14)

% v_body comparison
subplot(5,3,2)
hold on
plot(SM.t,SM.v,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.v,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$v_{b}$ (m/sec)','interpreter','latex','FontSize',14)

% w_body comparison
subplot(5,3,3)
hold on
plot(SM.t,SM.w,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.w,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$w_{b}$ (m/sec)','interpreter','latex','FontSize',14)

% p comparison
subplot(5,3,4)
hold on
plot(SM.t,SM.p,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.p,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$p$ (rad/sec)','interpreter','latex','FontSize',14)

% q comparison
subplot(5,3,5)
hold on
plot(SM.t,SM.q,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.q,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$q$ (rad/sec)','interpreter','latex','FontSize',14)

% r comparison
subplot(5,3,6)
hold on
plot(SM.t,SM.r,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.r,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$r$ (rad/sec)','interpreter','latex','FontSize',14)

% phi comparison
subplot(5,3,7)
hold on
plot(SM.t,SM.phi*180/pi,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.phi*180/pi,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$\phi$ (deg)','interpreter','latex','FontSize',14)

% theta comparison
subplot(5,3,8)
hold on
plot(SM.t,SM.theta*180/pi,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.theta*180/pi,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$\theta$ (deg)','interpreter','latex','FontSize',14)

% psi comparison
subplot(5,3,9)
hold on
plot(SM.t,SM.psi*180/pi,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.psi*180/pi,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$\psi$ (deg)','interpreter','latex','FontSize',14)

% x_Earth comparison
subplot(5,3,10)
hold on
plot(SM.t,SM.xE,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.xE,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$x_{E}$ (m)','interpreter','latex','FontSize',14)

% y_Earth comparison
subplot(5,3,11)
hold on
plot(SM.t,SM.yE,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.yE,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$y_{E}$ (m)','interpreter','latex','FontSize',14)

% z_Earth comparison
subplot(5,3,12)
hold on
plot(SM.t,SM.zE,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.zE,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$z_{E}$ (m)','interpreter','latex','FontSize',14)

% alpha comparison
subplot(5,3,13)
hold on
plot(SM.t,SM.alpha*180/pi,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.alpha*180/pi,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$\alpha$ (deg)','interpreter','latex','FontSize',14)

% beta comparison
subplot(5,3,14)
hold on
plot(SM.t,SM.beta*180/pi,'color','r','LineWidth',2.5,'DisplayName','Simulink')
plot(Aircraft_Data.time_vec,NL.beta*180/pi,'--','color','b','LineWidth',2,'DisplayName','Our RBD Solver')
hold off
xlabel('$t$ (sec)','interpreter','latex','FontSize',14)
ylabel('$\beta$ (deg)','interpreter','latex','FontSize',14)
legend('interpreter','latex','FontSize',14);

% Airplane's Trajectory in 3D
figure
plot3(SM.xE,SM.yE,-SM.zE,'r',NL.xE,NL.yE,-NL.zE,'--b','LineWidth',2.5)
grid on
title('Airplane Trajectory in 3D','interpreter','latex','FontSize',20)
xlabel('$x_{E}$ (m)','interpreter','latex','FontSize',14)
ylabel('$y_{E}$ (m)','interpreter','latex','FontSize',14)
zlabel('$h$ (m)','interpreter','latex','FontSize',14)
legend('Simulink','Our RBD Solver','Location','NorthEast','interpreter','latex','FontSize',14)