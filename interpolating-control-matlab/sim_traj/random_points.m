n_points_z = 1e4;
n_points_y = n_points_z;

load('data/model/uav_y_0.01_model_data.mat','InvSet');
InvSet_y = InvSet.copy();
load('data/model/uav_z_0.01_model_data.mat','InvSet');
InvSet_z = InvSet.copy();

points_y = zeros(InvSet_y.Dim,n_points_y);
points_z = zeros(InvSet_z.Dim,n_points_z);

parfor i=1:n_points_y
    points_y(:,i) = InvSet_y.randomPoint();
end

parfor i=1:n_points_z
    points_z(:,i) = InvSet_z.randomPoint();
end

save("output/random_points_"+n_points_y+".mat",'points_y','points_z','n_points_y','n_points_z');