cd ../..

cd sim
run get_sim_data
cd ..
cd results/scripts
run analyse_sim_output
run cost_sim_output
cd results/scripts
run cost_table
cd results/scripts
run print_table