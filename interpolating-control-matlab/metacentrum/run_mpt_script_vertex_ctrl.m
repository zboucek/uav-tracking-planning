cd tbxmanager;
tbxmanager restorepath;
cd ..
cd interpolating_control;
mkdir output;

MetaParPool('open');
run('get_vertex_control.m');

MetaParPool('close');
quit;
