%% calc criterion functions in time
cd ../..
model_folder = "data/model/";
file_suffix = "_0.01_room_crazyflie_model_data.mat";
load(model_folder + "uav_phi"+file_suffix);
addpath data/explicit_ic
addpath data/explicit_2ic
addpath data/explicit_mpc

load('data/vertex_ctrl/vertex_sets.mat')
load('data/vertex_ctrl/quad_y_vc.mat')
load('data/vertex_ctrl/quad_z_vc.mat')
load('data/vertex_ctrl/quad_phi_vc.mat')

Nall = length(InvSet_y.V)+length(InvSet_z.V)+length(InvSet_phi.V);
% files = {"sim_explicit_IC_tree_nonlinear","sim_explicit_2IC_tree_nonlinear","sim_explicit_MPC_tree_nonlinear"};
files = {"sim_explicit_IC_nonlinear","sim_explicit_2IC_nonlinear","sim_explicit_MPC_nonlinear"};

N = 501;
for j = 1:length(files)
index = [];
    Jphi_all = zeros(Nall,N);
    Jy_all = zeros(Nall,N);
    Jz_all = zeros(Nall,N);
    J_all = zeros(Nall,N);
    for n = 1:length(InvSet_y.V)
    for p = 1:length(InvSet_z.V)
%     for o = 1:length(InvSet_phi.V)
try
        load("sim/eval/"+files{j}+"_"+n+"_"+p+ "_eval.mat");
        Jphi_all(n+p-1,:) = Jphi;
        Jz_all(n+p-1,:) = Jz;
        Jy_all(n+p-1,:) = Jy;
        J_all(n+p-1,:) = J;
catch
        index = [index,n+p-1];
        warning('File does not exist!');
end
%     end
    end
    end
    
    % erase error cost
    J_all(index,:) = [];
    Jz_all(index,:) = [];
    Jy_all(index,:) = [];
    Jphi_all(index,:) = [];
    
    J_mean = zeros(1,N);
    Jy_mean = zeros(1,N);
    Jz_mean = zeros(1,N);
    Jphi_mean = zeros(1,N);
    J_std = zeros(1,N);
    Jy_std = zeros(1,N);
    Jz_std = zeros(1,N);
    Jphi_std = zeros(1,N);
    for i = 1:N
        J_mean(i) = mean(J_all(:,i));
        Jy_mean(i) = mean(Jy_all(:,i));
        Jz_mean(i) = mean(Jz_all(:,i));
        Jphi_mean(i) = mean(Jphi_all(:,i));
        J_std(i) = std(J_all(:,i));
        Jy_std(i) = std(Jy_all(:,i));
        Jz_std(i) = std(Jz_all(:,i));
        Jphi_std(i) = std(Jphi_all(:,i));
    end 
    save("results/"+files{j}+"_cost_mean.mat",'J_mean','Jy_mean','Jz_mean','Jphi_mean','J_std','Jy_std','Jz_std','Jphi_std','J_all','Jy_all','Jz_all','Jphi_all');
end