cd tbxmanager;
tbxmanager restorepath;
cd ..
cd interpolating_control;
mkdir output;

MetaParPool('open');
run('run_interpolating_control.m');

MetaParPool('close');
quit;
