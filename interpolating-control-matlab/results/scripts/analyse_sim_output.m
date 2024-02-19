%% evaluate criterion functions according to outputs of simulations from sim/get_sim_data.m
cd ../..
model_folder = "data/model/";
file_suffix = "_0.01_room_crazyflie_model_data.mat";
load(model_folder + "uav_phi"+file_suffix);
addpath data/explicit_ic
addpath data/explicit_2ic
addpath data/explicit_mpc

load("data/vertex_ctrl/vertex_sets.mat")
load("data/vertex_ctrl/quad_y_vc.mat")
load("data/vertex_ctrl/quad_z_vc.mat")
load("data/vertex_ctrl/quad_phi_vc.mat")

model_folder = "data/model/";
file_suffix = "_0.01_room_crazyflie_model_data.mat";

load(model_folder + "uav_phi"+file_suffix,"Q","R");
Qphi = Q;
Rphi = R;

load(model_folder + "uav_y"+file_suffix,"Q","R");
Qy = Q;
Ry = R;

load(model_folder + "uav_z"+file_suffix,"Q","R");
Qz= Q;
Rz = R;

% Q = diag([1/0.5^2,1/0.5^2,1/deg2rad(10)^2,1/(0.5*vy_max)^2,1/(0.5*vz_max)^2,1/deg2rad(100)^2]);
Q = diag([Qy(1,1), Qz(1,1), 0*Qphi(1,1), Qy(2,2), Qz(2,2), 0*Qphi(2,2)]);
R = diag([Ry, Rz]);

mkdir sim/eval
% files = {"sim_explicit_IC_tree_nonlinear","sim_explicit_2IC_tree_nonlinear","sim_explicit_MPC_tree_nonlinear"};
files = {"sim_explicit_IC_nonlinear","sim_explicit_2IC_nonlinear","sim_explicit_MPC_nonlinear"};
% files = {"sim_explicit_MPC_tree_nonlinear"};
n =1; p=1; o=1;
for j = 1:length(files)
for n = 1:length(InvSet_y.V)
for p = 1:length(InvSet_z.V)
% for o = 1:length(InvSet_phi.V)
try
    load("sim/output/"+files{j}+"_"+n+"_"+p+"_out.mat");
    Jphi = zeros(1,501);
    Jy = zeros(1,501);
    Jz = zeros(1,501);
    J = zeros(1,501);
    for i = 1:501
%         Jphi(i) = x(i,[3,6],1)*Qphi*x(i,[3,6],1)' + u(i,2,1)*Rphi*u(i,2,1)';
%         Jphi(i) = 0
        u_phi = 0;
        Jz(i) = x(i,[2,5],1)*Qz*x(i,[2,5],1)' + u(i,2,1)*Rz*u(i,2,1)';
        if i == 1
            u_phi = 0;
        else
            if files{j} == "sim_explicit_IC_nonlinear"
%             if files{j} == "sim_explicit_IC_tree_nonlinear"
%                 u_phi = quad_ic_tree_y(x(i,[1,4],1));
                u_phi = quad_ic_y(x(i,[1,4],1));
%             elseif files{j} == "sim_explicit_2IC_tree_nonlinear"
            elseif files{j} == "sim_explicit_2IC_nonlinear"
%                 u_phi = quad_2ic_tree_y(x(i,[1,4],1));
                u_phi = quad_2ic_y(x(i,[1,4],1));
            else
                u_phi = quad_mpc_y(x(i,[1,4],1));
%                 u_phi = quad_mpc_tree_y(x(i,[1,4],1));
            end
        end
        Jy(i) = x(i,[1,4],1)*Qy*x(i,[1,4],1)' + u(i,1,1)*Ry*u(i,1,1)';
        J(i) = x(i,:,1)*Q*x(i,:,1)' + u(i,:,1)*R*u(i,:,1)';
    end
    save("sim/eval/"+files{j}+"_"+n+"_"+p+"_eval.mat",'Jphi','Jz','Jy','J');    
% end
catch
    warning('File does not exist!');
end
end
end
end

cd results/scripts