clear; clc;
cd ../..

% folder_in = "results/data/";
folder_out = "results/";


load("data/vertex_ctrl/vertex_sets.mat")
control = ["mpc","icr","lq2icr"];
files = {"sim_explicit_MPC_nonlinear","sim_explicit_IC_nonlinear","sim_explicit_2IC_nonlinear"};
load("sim/output/"+files{1}+"_"+1+"_"+1+"_out.mat");
ncontrol = length(files);
[nk,nx] = size(x);
nyV = length(InvSet_y.V);
nzV = length(InvSet_z.V);
nv = length(InvSet_y.V)*length(InvSet_z.V);
nmodels = 2;
nu = size(u,2);
energy = zeros(nu,nv,ncontrol,nmodels+1);
ise = zeros(1,nv,ncontrol,nmodels+1);
mean_ise = zeros(1,ncontrol,nmodels+1);
mean_energy = zeros(1,ncontrol,nmodels+1);
std_ise = zeros(1,ncontrol,nmodels+1);
std_energy = zeros(1,ncontrol,nmodels+1);
perc_ise = zeros(1,ncontrol,nmodels+1);
perc_energy = zeros(1,ncontrol,nmodels+1);
for i = 1:ncontrol
    for n = 1:nyV
        for p = 1:nzV
            load("sim/output/"+files{i}+"_"+n+"_"+p+"_out.mat");
            for k = 1:nmodels
                ise(:,(n-1)*nyV+p,i,k) = sum((x(:,k)).^2,1)*0.01;
                energy(:,(n-1)*nyV+p,i,k) = sum(u(:,k).^2,1)*0.01;
            end
            ise(:,(n-1)*nyV+p,i,nmodels+1) = sum(ise(:,(n-1)*nyV+p,i,1:2));
            energy(:,(n-1)*nyV+p,i,nmodels+1) = sum(energy(:,(n-1)*nyV+p,i,1:2));
        end
    end
end

format_num = 5;
for i = 1:ncontrol
    for k = 1:nmodels+1
        mean_ise(:,i,k) = round(mean(ise(:,:,i,k)),format_num,'significant');
        mean_energy(:,i,k) = round(mean(sum(energy(:,:,i,k))),format_num,'significant');
        std_ise(:,i,k) = round(std(ise(:,:,i,k)),format_num,'significant');
        std_energy(:,i,k) = round(std(sum(energy(:,:,i,k))),format_num,'significant');
        if i>1
            perc_ise(1,i,k) = round(100*(mean_ise(:,i,k)/mean_ise(:,1,k)-1),2);
            perc_energy(1,i,k) = round(100*(mean_energy(:,i,k)/mean_energy(:,1,k)-1),2);
        end
    end
end

save(folder_out + "eval_ise_energy","mean_energy","mean_ise","std_energy","std_ise","perc_energy","perc_ise");

disp("ISE");
[mean_ise;perc_ise;std_ise]
disp("energy");
[mean_energy;perc_energy;std_energy]