% vypocita 50 eMPC regulatoru s horizontem 1:50 a ulozi jejich funkce do
% y
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

run('uav_y_model.m');
folder = "data/explicit_mpc/y"; 
mkdir(folder);
getMPCforN(system,folder+"/y")

MetaParPool('close');
% quit;
