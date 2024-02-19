cd ../..

folder_in = "results/";
folder_out_table = "results/";
mkdir(folder_out_table);

name = {"ic","2ic","mpc"};

n_dec = 5;
n_perc = 2;

N = 3;
table = cell(length(name),4);
for i = 1:length(name)/N
    table{i,1} = name{i};
    load(folder_in+"cost"+name{i});
    table{i,2} = round(J_table_mean,n_dec,'significant');
    table{i,3} = round(J_table_var,n_dec,'significant');
    N2 = length(name)/N+i;
    load(folder_in+"cost"+name{N2});
    table{N2,1} = name{N2};
    table{N2,2} = round(J_table_mean,n_dec,'significant');
    table{N2,3} = round(J_table_var,n_dec,'significant');
    N3 = 2*(length(name)/N)+i;
    load(folder_in+"cost"+name{N3});
    table{N3,1} = name{N3};
    table{N3,2} = round(J_table_mean,n_dec,'significant');
    table{N3,3} = round(J_table_var,n_dec,'significant');
    table{i,4} = round(((table{i,2}/table{N3,2})-1)*100,n_perc,'decimals');
    table{N2,4} = round(((table{N2,2}/table{N3,2})-1)*100,n_perc,'decimals');
end

save(folder_out_table+"table",'table');

table_y = cell(length(name),4);
for i = 1:length(name)/N
    table_y{i,1} = name{i};
    load(folder_in+"cost"+name{i});
    table_y{i,2} = round(Jy_table_mean,n_dec,'significant');
    table_y{i,3} = round(Jy_table_var,n_dec,'significant');
    N2 = length(name)/N+i;
    load(folder_in+"cost"+name{N2});
    table_y{N2,1} = name{N2};
    table_y{N2,2} = round(Jy_table_mean,n_dec,'significant');
    table_y{N2,3} = round(Jy_table_var,n_dec,'significant');
    N3 = 2*(length(name)/N)+i;
    load(folder_in+"cost"+name{N3});
    table_y{N3,1} = name{N3};
    table_y{N3,2} = round(Jy_table_mean,n_dec,'significant');
    table_y{N3,3} = round(Jy_table_var,n_dec,'significant');
    table_y{i,4} = round(((table_y{i,2}/table_y{N3,2})-1)*100,n_perc,'decimals');
    table_y{N2,4} = round(((table_y{N2,2}/table_y{N3,2})-1)*100,n_perc,'decimals');
end

save(folder_out_table+"table_y",'table_y');

table_z = cell(length(name),4);
for i = 1:length(name)/N
    table_z{i,1} = name{i};
    load(folder_in+"cost"+name{i});
    table_z{i,2} = round(Jz_table_mean,n_dec,'significant');
    table_z{i,3} = round(Jz_table_var,n_dec,'significant');
    N2 = length(name)/N+i;
    load(folder_in+"cost"+name{N2});
    table_z{N2,1} = name{N2};
    table_z{N2,2} = round(Jz_table_mean,n_dec,'significant');
    table_z{N2,3} = round(Jz_table_var,n_dec,'significant');
    N3 = 2*(length(name)/N)+i;
    load(folder_in+"cost"+name{N3});
    table_z{N3,1} = name{N3};
    table_z{N3,2} = round(Jz_table_mean,n_dec,'significant');
    table_z{N3,3} = round(Jz_table_var,n_dec,'significant');
    table_z{i,4} = round(((table_z{i,2}/table_z{N3,2})-1)*100,n_perc,'decimals');
    table_z{N2,4} = round(((table_z{N2,2}/table_z{N3,2})-1)*100,n_perc,'decimals');
end

save(folder_out_table+"table_z",'table_z');
% 
% table_phi = cell(length(name),4);
% for i = 1:length(name)/N
%     table_phi{i,1} = name{i};
%     load(folder_in+"cost"+name{i});
%     table_phi{i,2} = round(Jphi_table_mean,n_dec,'significant');
%     table_phi{i,3} = round(Jphi_table_var,n_dec,'significant');
%     N2 = length(name)/N+i;
%     load(folder_in+"cost"+name{N2});
%     table_phi{N2,1} = name{N2};
%     table_phi{N2,2} = round(Jphi_table_mean,n_dec,'significant');
%     table_phi{N2,3} = round(Jphi_table_var,n_dec,'significant');
%     N3 = 2*(length(name)/N)+i;
%     load(folder_in+"cost"+name{N3});
%     table_phi{N3,1} = name{N3};
%     table_phi{N3,2} = round(Jphi_table_mean,n_dec,'significant');
%     table_phi{N3,3} = round(Jphi_table_var,n_dec,'significant');
%     table_phi{i,4} = round(((table_phi{i,2}/table_phi{N3,2})-1)*100,n_perc,'decimals');
%     table_phi{N2,4} = round(((table_phi{N2,2}/table_phi{N3,2})-1)*100,n_perc,'decimals');
% end
% 
% save(folder_out_table+"table_phi",'table_phi');

cd results/scripts/