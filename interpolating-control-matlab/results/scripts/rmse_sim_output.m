cd ../..
run uav_phi_model
addpath data/explicit_ic
addpath data/explicit_mpc

load('data/vertex_ctrl/vertex_sets.mat')
load('data/vertex_ctrl/quad_y_vc.mat')
load('data/vertex_ctrl/quad_z_vc.mat')
load('data/vertex_ctrl/quad_phi_vc.mat')

files = {"sim_explicit_IC_tree_nonlinear","sim_explicit_MPC_tree_nonlinear"};
name = {"ic","mpc"};
N = 501;
Ny  = length(InvSet_y.V);
Nz = length(InvSet_z.V);
Nphi = length(InvSet_phi.V);
Nall = Ny*Nz*Nphi*N;
for j = 1:length(files)
    rmse = zeros(1,Nall);
    for n = 1:Ny
    for p = 1:Nz
    for o = 1:Nphi
        load("sim/output/"+files{j}+"_"+n+"_"+p+"_"+o + "_out.mat");
        for i = 1:N
            rmse(n*p*o*i) = sqrt(0.5*(x(i,1,1)*x(i,1,1)+x(i,2,1)*x(i,2,1)));
        end
    end
    end
    end
    rmsemean = (1/length(rmse))*sum(rmse);
    rmsestd = std(rmse);
    save("results/"+name{j}+"_rmse.mat",'rmse','rmsemean','rmsestd');    
end

cd results/scripts