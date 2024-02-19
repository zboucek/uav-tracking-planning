%% prints plot of functions for UAV controller IC, 2IC and MPC 
cd ../..
close all; clc; clear;
addpath('data/explicit_ic')
addpath('data/explicit_2ic')
addpath('data/explicit_mpc')

edges = 0;
if edges == 0
    mkdir('results/plots/control_funs/no_edges');
    folder = "results/plots/control_funs/no_edges/";
else
    mkdir('results/plots/control_funs');
    folder = "results/plots/control_funs/";
end

load('printable_colors.mat');
printpar = "-dpdf";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no


% load('ic_phi.mat');
% fig = plot_union(eIC_phi,color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% view([-135 15])
% printPlot(fig, "control_ic_phi", folder, printbool, printpar, savebool);
% 
% load('2ic_phi.mat');
% fig = plot_union(e2IC,color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% view([-135 15])
% printPlot(fig, "control_2ic_phi", folder, printbool, printpar, savebool);
% 
% load('quad_expmpc_ctrl_phi.mat');
% fig = plot_union(expmpc_phi.feedback.slice([3 4],[0 0]),color4,edges);
% xlabel('\phi [rad]');
% ylabel('\omega [rad/s]');
% zlabel('\tau [N⋅m]');
% view([-135 15])
% printPlot(fig, "control_mpc_phi", folder, printbool, printpar, savebool);

load('get_quad2IC_output.mat');
load('get_quadIC_output.mat');
load('get_quadMPC_output.mat');

fig = plot_union(eIC_y,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
view([135 40])
printPlot(fig, "control_ic_y", folder, printbool, printpar, savebool);

fig = plot_union(e2IC_y,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
view([135 40])
printPlot(fig, "control_2ic_y", folder, printbool, printpar, savebool);

fig = plot_union(expmpc_y.feedback,color4,edges);
xlabel('y [m]');
ylabel('v_y [m/s]');
zlabel('a_y [m/s^2]');
view([135 40])
printPlot(fig, "control_mpc_y", folder, printbool, printpar, savebool);

fig = plot_union(eIC_z,color4,edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
view([135 40])
printPlot(fig, "control_ic_z", folder, printbool, printpar, savebool);

fig = plot_union(e2IC_z,[color4;color3],edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
view([135 40])
printPlot(fig, "control_2ic_z", folder, printbool, printpar, savebool);

fig = plot_union(expmpc_z.feedback,[color4;color3],edges);
xlabel('z [m]');
ylabel('v_z [m/s]');
zlabel('a_z [m/s^2]');
view([135 40])
printPlot(fig, "control_mpc_z", folder, printbool, printpar, savebool);

cd results/scripts