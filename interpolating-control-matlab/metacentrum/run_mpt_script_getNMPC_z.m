% vypocita 50 eMPC regulatoru s horizontem 1:50 a ulozi jejich funkce do
% z
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

run('uav_z_model.m');
folder = "data/explicit_mpc/z"; 
mkdir(folder);
getMPCforN(system,folder+"/z")

MetaParPool('close');
% quit;
