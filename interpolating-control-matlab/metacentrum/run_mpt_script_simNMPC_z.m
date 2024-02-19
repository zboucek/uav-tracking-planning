% vypocita 50 eMPC regulatoru s horizontem 1:50 a ulozi jejich funkce do
% phi
cd tbxmanager;
tbxmanager restorepath;
cd ..
cd interpolating_control;
% mkdir output;

cd sim
run('get_sim_data_mpc_z.m');

% quit;