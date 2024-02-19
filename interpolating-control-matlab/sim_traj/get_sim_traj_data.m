%% do the simulations for all vertices of y and z control for MPC, IC and 2IC and save to file in folder output

run init_sim.m

mkdir output

refs_y = {refStepTrajGen(t_y, Ts_y, 2*Nlq_y),refSineTrajGen(t_y, Ts_y, 2*Nlq_y)};
refs_z = {refStepTrajGen(t_z, Ts_z, 2*Nlq_z),refSineTrajGen(t_z, Ts_z, 2*Nlq_z)};

files = ["sim_traj_ic","sim_traj_mpc","sim_traj_mpc_block"];

for j = 1:length(files)
    for n = 1:length(refs_y)
        ref_y = refs_y{n};
        ref_z = refs_z{n};
        try
            sim(files{j});
            save("output/"+files{j}+"_"+n+"_out.mat",'state','control','comp_time_y','comp_time_z','ref_y','ref_z','control_y','control_z','control_phi');
        catch
            warning('Simulation ended with an error!');
        end
    end
end