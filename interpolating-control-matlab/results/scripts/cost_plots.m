%% plot criterion functions in time for each controller
cd ../..
load results/sim_explicit_IC_nonlinear_cost_mean
% load results/sim_explicit_IC_tree_nonlinear_cost_mean
load printable_colors.mat
printpar = "-dpdf";
folder = "results/plots/cost/";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no

fig_ic_J = figure;
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf]);
printPlot(fig_ic_J, "cost_ic", folder, printbool, printpar, savebool);

fig_ic_Jphi = figure;
shadedErrorBar(((1:501)-1)*0.1,Jphi_mean,Jphi_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_ic_Jphi, "cost_ic_phi", folder, printbool, printpar, savebool);

fig_ic_Jy = figure;
shadedErrorBar(((1:501)-1)*0.1,Jy_mean,Jy_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_ic_Jy, "cost_ic_y", folder, printbool, printpar, savebool);

fig_ic_Jz = figure;
shadedErrorBar(((1:501)-1)*0.1,Jz_mean,Jz_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_ic_Jz, "cost_ic_z", folder, printbool, printpar, savebool);

load results/sim_explicit_2IC_nonlinear_cost_mean
% load results/sim_explicit_2IC_tree_nonlinear_cost_mean


fig_2ic_J = figure;
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf]);
printPlot(fig_2ic_J, "cost_2ic", folder, printbool, printpar, savebool);

fig_2ic_Jphi = figure;
shadedErrorBar(((1:501)-1)*0.1,Jphi_mean,Jphi_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_2ic_Jphi, "cost_2ic_phi", folder, printbool, printpar, savebool);

fig_2ic_Jy = figure;
shadedErrorBar(((1:501)-1)*0.1,Jy_mean,Jy_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_2ic_Jy, "cost_2ic_y", folder, printbool, printpar, savebool);

fig_2ic_Jz = figure;
shadedErrorBar(((1:501)-1)*0.1,Jz_mean,Jz_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_2ic_Jz, "cost_2ic_z", folder, printbool, printpar, savebool);

load results/sim_explicit_MPC_nonlinear_cost_mean
% load results/sim_explicit_MPC_tree_nonlinear_cost_mean

fig_mpc_J = figure;
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_mpc_J, "cost_mpc", folder, printbool, printpar, savebool);

fig_mpc_Jphi = figure;
shadedErrorBar(((1:501)-1)*0.1,Jphi_mean,Jphi_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_mpc_Jphi, "cost_mpc_phi", folder, printbool, printpar, savebool);

fig_mpc_Jy = figure;
shadedErrorBar(((1:501)-1)*0.1,Jy_mean,Jy_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_mpc_Jy, "cost_mpc_y", folder, printbool, printpar, savebool);

fig_mpc_Jz = figure;
shadedErrorBar(((1:501)-1)*0.1,Jz_mean,Jz_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_mpc_Jz, "cost_mpc_z", folder, printbool, printpar, savebool);

cd results/scripts