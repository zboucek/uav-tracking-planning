%% prints plot of partition for UAV controller IC, 2IC and MPC 
cd ../..
close all; clc; clear;
load('printable_colors.mat');
addpath('data/explicit_ic')
addpath('data/explicit_2ic')
addpath('data/explicit_mpc')

edges = 0;
if edges == 0
    mkdir('results/plots/partitions/no_edges');
    folder = "results/plots/partitions/no_edges/";
else
    mkdir('results/plots/partitions');
    folder = "results/plots/partitions/";
end
    
printpar = "-dpdf";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no
% 
% load('ic_phi.mat');
% fig = plot_union_part(eIC_phi,color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% printPlot(fig, "partition_ic_phi", folder, printbool, printpar, savebool);
% 
% load('2ic_phi.mat');
% fig = plot_union_part(e2IC,color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% printPlot(fig, "partition_2ic_phi", folder, printbool, printpar, savebool);
% 
% load('quad_expmpc_ctrl_phi.mat');
% fig = plot_union_part(expmpc_phi.partition.slice([3 4],[0 0]),color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% printPlot(fig, "partition_mpc_phi", folder, printbool, printpar, savebool);

load('get_quad2IC_output.mat');
load('get_quadIC_output.mat');
load('get_quadMPC_output.mat');

fig = plot_union_part(eIC_y,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
printPlot(fig, "partition_ic_y", folder, printbool, printpar, savebool);

fig = plot_union_part(e2IC_y,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
printPlot(fig, "partition_2ic_y", folder, printbool, printpar, savebool);

fig = plot_union_part(expmpc_y.partition,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
printPlot(fig, "partition_mpc_y", folder, printbool, printpar, savebool);

fig = plot_union_part(eIC_z,[color4;color3],edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
printPlot(fig, "partition_ic_z", folder, printbool, printpar, savebool);

fig = plot_union_part(e2IC_z,[color4;color3],edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
printPlot(fig, "partition_2ic_z", folder, printbool, printpar, savebool);

fig = plot_union_part(expmpc_z.partition,[color4;color3],edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
printPlot(fig, "partition_mpc_z", folder, printbool, printpar, savebool);

cd results/scripts