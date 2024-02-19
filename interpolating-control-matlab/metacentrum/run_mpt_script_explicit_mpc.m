cd tbxmanager;
tbxmanager restorepath;
cd ..
cd interpolating_control;
% mkdir output;

set(groot,'defaultFigureVisible','off')
if (MetaParPool('open') <= 0)
	disp(['ERROR: Unable to initialize MetaParPool! Exiting...']);
	exit(1);
end


cd data/explicit_mpc
run('get_eMPC.m');
cd data/explicit_mpc
run('mpc2tree.m');


MetaParPool('close');
% quit;
