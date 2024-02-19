cd ../..
load printable_colors.mat
printpar = "-dpdf";
folder = "results/test/plots/";
printbool = "print"; % no, export, print
savebool = 1; %1 yes, 0 no

load('results/test/ic_easy_cost_mean');

fig_ic_J = figure;
shadedErrorBar(0:29,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf]);
printPlot(fig_ic_J, "cost_ic", folder, printbool, printpar, savebool);

load('results/test/ic2_easy_cost_mean');

fig_ic_J2 = figure;
shadedErrorBar(0:29,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf]);
printPlot(fig_ic_J2, "cost_ic2", folder, printbool, printpar, savebool);

load('results/test/mpc_easy_cost_mean');

fig_mpc_J = figure;
shadedErrorBar(0:29,J_mean,J_std*2,struct('color',color4(5,:)),1);
xlabel('t[s]');
ylabel('cost');
ylim([0 inf])
printPlot(fig_mpc_J, "cost_mpc", folder, printbool, printpar, savebool);

cd results/scripts