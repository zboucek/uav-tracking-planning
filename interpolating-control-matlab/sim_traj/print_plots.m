%% prints plot runs for controller IC, eIC and MPC
cd ..
close all; clc; clear;
load('printable_colors.mat');
% set(0,'defaultAxesFontName', 'Helvetica')
% set(0,'defaultTextFontName', 'Helvetica')

% setpoint = [5;0];

edges = 1;
printpar = "-dpdf";
folder = "sim_traj/plots/";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no
mkdir(folder)

saveResults = 1;
% type = ["sim_traj_mpc","sim_traj_ic"];
type = ["sim_traj_mpc","sim_traj_mpc_block","sim_traj_ic"];
file = ["step","sinewave"];
folder_output = folder;
folder_in = "sim_traj/output/";

style = {'-','--',':','-.'};
names = {'MPC','MPC block','IC'};

i = 1;
for m = 1:length(file)
    % y position plot
    fig = figure(i);
    for n = 1:length(type)
        load(folder_in+type{n}+"_"+m+"_out.mat");
        xkForPrint(:,:,n) = state.signals.values;
        ukForPrint(:,:,n) = control.signals.values;
        uk_phiForPrint(:,:,n) = control_y.signals.values;
    end
    ref_y = ref_y(1,1:round(length(ref_y)/2));
    ref_z = ref_z(1,1:round(length(ref_z)/2));
    stairs(linspace(state.time(1),state.time(end),length(ref_y)),ref_y)
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,1,n));
    end
    xlabel('t [s]');
    ylabel('y [m]');
%     ylim([-10 10]);
    legend('r',names{1},names{2},names{3});
    hold off;
    printPlot(fig, "y_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;

    % z position plot
    fig = figure(i);
    stairs(linspace(state.time(1),state.time(end),length(ref_z)),ref_z)
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,2,n));
    end
    xlabel('t [s]');
    ylabel('z [m]');
%     ylim([-5 5]);
    legend('r',names{1},names{2},names{3});
    hold off;
    printPlot(fig, "z_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % y velocity plot
    fig = figure(i);
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,4,n));
    end
    xlabel('t [s]');
    ylabel('\dot{y} [m\cdot s^{-1}]')
%     ylim([-5 5]);
    legend(names{1},names{2},names{3});
    hold off;
    printPlot(fig, "vel_y_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % z velocity plot
    fig = figure(i);
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,5,n));
    end
    xlabel('t [s]');
    ylabel('\dot{z} [m\cdot s^{-1}]')
%     ylim([-5 5]);
    legend(names{1},names{2},names{3});
    hold off;
    printPlot(fig, "vel_z_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % angle plot
    fig = figure(i);
    stairs(control_y.time,uk_phiForPrint(:,1,n))
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,3,n));
    end
    xlabel('t [s]');
    ylabel('\phi [rad]');
%     ylim([-5 5]);
    legend('r',names{1},names{2},names{3});
    hold off;
    printPlot(fig, "angle_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % angular rate plot
    fig = figure(i);
    hold on;
    for n = 1:length(type)
        stairs(state.time,xkForPrint(:,6,n));
    end
    xlabel('t [s]');
    ylabel('\omega [rad\cdot s^{-1}]');
%     ylim([-5 5]);
    legend(names{1},names{2},names{3});
    hold off;
    printPlot(fig, "arate_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % thrust plot
    fig = figure(i);
    hold on;
    for n = 1:length(type)
        stairs(control.time,ukForPrint(:,1,n));
    end
    xlabel('t [s]');
    ylabel('F^B_T [N]');
%     ylim([-5 5]);
    legend(names{1},names{2},names{3});
    hold off;
    printPlot(fig, "thrust_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
    
    % torque plot
    fig = figure(i);
    hold on;
    for n = 1:length(type)
        stairs(control.time,ukForPrint(:,2,n));
    end
    xlabel('t [s]');
    ylabel('\tau_{R_x} [N\cdot m^{-1}]');
%     ylim([-5 5]);
    legend(names{1},names{2},names{3});
    hold off;
    printPlot(fig, "torque_"+type{n}+"_"+file{m}, folder, printbool, printpar, savebool);
    i = i + 1;
% 
%     fig = figure(i);
%     hold on;
%     for n = 1:length(control)
%         stairs(ukForPrint(1,:,n),style{n});
%     end
%     xlabel('k');
%     ylabel('u');
%     ylim([-1 1]);
%     legend(names{1},names{2},names{3});
%     hold off;
%     printPlot(fig, "uk_"+control{n}+"_"+file{m}+type{o}+"_"+solvers{p}, folder, printbool, printpar, savebool);
%     i = i + 1;
% 
%     fig = figure(i);
%     hold on;
%     stairs(ck,style{1});
%     xlabel('k');
%     ylabel('c');
%     ylim([0 1]);
%     hold off;
%     printPlot(fig, "c_"+control{n}+"_"+file{m}+type{o}+"_"+solvers{p}, folder, printbool, printpar, savebool);
%     i = i + 1;
% 
%     fig = figure(i);
%     hold on;
%     stairs(c1k,style{1});
%     stairs(c2k,style{2});
%     xlabel('k');
%     ylabel('c');
%     ylim([0 1]);
%     legend('c_1','c_2');
%     hold off;
%     printPlot(fig, "c12_"+control{n}+"_"+file{m}+type{o}+"_"+solvers{p}, folder, printbool, printpar, savebool);
%     i = i + 1;
end

cd sim_traj