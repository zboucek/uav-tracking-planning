%% calc and save tabular values of criterions for all controllers
cd ../..
files = {"sim_explicit_IC_nonlinear","sim_explicit_2IC_nonlinear","sim_explicit_MPC_nonlinear"};
% files = {"sim_explicit_IC_tree_nonlinear","sim_explicit_2IC_tree_nonlinear","sim_explicit_MPC_tree_nonlinear"};
% name = {"ic_tree","2ic_tree","mpc_tree"};
name = {"ic","2ic","mpc"};

for j=1:length(files)
    load("results/"+files{j}+"_cost_mean.mat")
    J_table_mean = mean(sum(J_all,2));
    J_table_var = var(sum(J_all,2));
    Jy_table_mean = mean(sum(Jy_all,2));
    Jy_table_var = var(sum(Jy_all,2));
    Jz_table_mean = mean(sum(Jz_all,2));
    Jz_table_var = var(sum(Jz_all,2));
    Jphi_table_mean = mean(sum(Jphi_all,2));
    Jphi_table_var = var(sum(Jphi_all,2));
    
    save("results/cost"+name{j}+".mat",'J_table_mean','J_table_var','Jy_table_mean','Jy_table_var','Jz_table_mean','Jz_table_var','Jphi_table_mean','Jphi_table_var')
end