%% prints figure with comparison of criterion functions for MPC, IC and 2IC
cd ../..
load results/sim_explicit_IC_nonlinear_cost_mean
% load results/sim_explicit_IC_tree_nonlinear_cost_mean
load printable_colors.mat
printpar = "-dpdf";
folder = "results/plots/cost/";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no

fig_ic_J = figure;
hold on;
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf]);

load results/sim_explicit_2IC_nonlinear_cost_mean
% load results/sim_explicit_2IC_tree_nonlinear_cost_mean
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color4(1,:)),1);

load results/sim_explicit_MPC_nonlinear_cost_mean
% load results/sim_explicit_MPC_tree_nonlinear_cost_mean
shadedErrorBar(((1:501)-1)*0.1,J_mean,J_std*2,struct('color',color1(5,:)),1);

printPlot(fig_ic_J, "cost_ic_2ic_mpc", folder, printbool, printpar, savebool);
legend('IC','eIC','MPC')
cd results/scripts