%% do the computational time evaluation of y and z control for MPC and IC and save to file in folder output

run init_sim.m

mkdir output

ref_y = refStepTrajGen(t_y, Ts_y, 2*Nlq_y);
ref_z = refStepTrajGen(t_z, Ts_z, 2*Nlq_z);

load("output/random_points_10000.mat")
load('data/model/lqr_y_0.01_params.mat','Nlq')
Nlq_y = Nlq;
load('data/model/lqr_z_0.01_params.mat','Nlq')
Nlq_z = Nlq;

t_y = zeros(1,n_points_y);
t_z = zeros(1,n_points_z);

k_y = randi(Nlq_y,1,n_points_y);
k_z = randi(Nlq_z,1,n_points_z);
for j = 1:n_points_y
    tstarty = tic;
    mpc_y(points_y(:,j),reshape(ref_y(:,k_y:k_y+Nlq_y-1),[],1));
    t_y(j) = toc(tstarty);
end
for j = 1:n_points_z
    tstartz = tic;
    mpc_z(points_z(:,j),reshape(ref_z(:,k_z:k_z+Nlq_z-1),[],1));
    t_z(j) = toc(tstartz);
end
save("output/time_mpc_out.mat",'t_y','t_z');

for j = 1:n_points_y
    tstarty = tic; 
    ic_y(points_y(:,j),reshape(ref_y(:,k_y:k_y+Nlq_y-1),[],1));
    t_y(j) = toc(tstarty);
end
for j = 1:n_points_z
    tstartz = tic;
    ic_z(points_z(:,j),reshape(ref_z(:,k_z:k_z+Nlq_z-1),[],1));
    t_z(j) = toc(tstartz);
end
save("output/time_ic_out.mat",'t_y','t_z');

for j = 1:n_points_y
    tstarty = tic;
    mpc_block_y(points_y(:,j),reshape(ref_y(:,k_y:k_y+Nlq_y-1),[],1));
    t_y(j) = toc(tstarty);
end
for j = 1:n_points_z
    tstartz = tic;
    mpc_block_z(points_z(:,j),reshape(ref_z(:,k_z:k_z+Nlq_z-1),[],1));
    t_z(j) = toc(tstartz);
end
save("output/time_mpc_block_out.mat",'t_y','t_z');