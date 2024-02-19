%% prints plot runs for controller IC, eIC and MPC
cd ..
close all; clc; clear;
load('printable_colors.mat');
% set(0,'defaultAxesFontName', 'Helvetica')
% set(0,'defaultTextFontName', 'Helvetica')

edges = 1;
printpar = "-dpdf";
folder = "sim_traj/plots/";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no
mkdir(folder)

saveResults = 1;
folder_output = folder;
folder_in = "sim_traj/output/";

i=1;
fig = figure(i);
load(folder_in + "time_ic_out.mat");
t_ic= t_y;
load(folder_in + "time_mpc_block_out.mat");
t_mpc_b= t_y;
load(folder_in + "time_mpc_out.mat");
t_mpc= t_y;
boxplot([t_mpc;t_mpc_b;t_ic]','Notch','on','Labels',{'MPC','MPC block','IC'},'Whisker',1);
ylabel('T [s]')
printPlot(fig, "time_box_plot", folder, printbool, printpar, savebool);
i = i+1;

cd sim_traj