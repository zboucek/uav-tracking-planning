%% do the simulations for all vertices of y and z control for MPC, IC and 2IC and save to file in folder output

run init_sim.m

mkdir output

% files = {"sim_explicit_IC_tree_nonlinear","sim_explicit_2IC_tree_nonlinear","sim_explicit_MPC_tree_nonlinear"};
% 
% for j = 1:length(files)
% for n = 1:length(InvSet_y.V)
% for p = 1:length(InvSet_z.V)
% for o = 1:length(InvSet_phi.V)
%     x0 = [InvSet_y.V(n,1)'; InvSet_z.V(p,1)'; InvSet_phi.V(o,1)'; InvSet_y.V(n,2)'; InvSet_z.V(p,2)'; InvSet_phi.V(o,2)'] ;
%     sim(files{j});
%     x = state.signals.values;
%     u = control.signals.values;
% 	save("output/"+files{j}+"_"+n+"_"+p+"_"+o + "_out.mat",'x','u');
% end
% end
% end
% end

Nsample = 501;
Nall = length(InvSet_y.V)+length(InvSet_z.V);
% files = {"sim_explicit_MPC_tree_nonlinear","sim_explicit_IC_tree_nonlinear","sim_explicit_2IC_tree_nonlinear"};
files = {"sim_explicit_MPC_nonlinear","sim_explicit_IC_nonlinear","sim_explicit_2IC_nonlinear"};
x = zeros(Nsample,6);
u = zeros(Nsample,2);
x0 = zeros(6,1);

for j = 1:length(files)
    for n = 1:length(InvSet_y.V)
        x0(1) = 0.9*InvSet_y.V(n,1)';
        for p = 1:length(InvSet_z.V)
            x0(2) = 0.9*InvSet_z.V(p,1)';
            try
                sim(files{j});
                x = state.signals.values;
                u = control.signals.values;
                save("output/"+files{j}+"_"+n+"_"+p + "_out.mat",'x','u');
            catch
                warning('Simulation ended with an error!');
            end
        end
    end
end