%% test MPCs for y control with different horizons on nonlinear model
run init_sim.m

mkdir y_output
cd ..
addpath('data/explicit_mpc/eMPC_y');
cd sim
Nsample = 501;

Nall = length(InvSet_y.V)+length(InvSet_z.V);%+length(InvSet_phi.V);
N = 50;
mpc_z = @(x) quad_mpc_tree_z(x);
mpc_phi = @(x) quad_mpc_tree_phi(x);
for i = 1:N
c = "y_mpc_"+i;
mpc_y = str2func(c);

x = zeros(Nsample,6,Nall);
u = zeros(Nsample,2,Nall);
x0 = zeros(6,1);
for n = 1:length(InvSet_y.V)
	x0(1) = InvSet_y.V(n,1)';
%     x0(4) = InvSet_y.V(n,2)';
    for p = 1:length(InvSet_y.V)
        x0(2) = InvSet_z.V(p,1)';
%         x0(5) = InvSet_z.V(p,2)';
        try
            sim("sim_explicit_MPC_N");
            x = state.signals.values;
            u = control.signals.values;
            save("y_output/"+"sim_explicit_MPC_y"+i+"__"+n+"_"+p + "_out.mat",'x','u');
        catch
            warning('Simulation ended with an error!');
        end
    end
end
end