clc; clear;
name = ["phi","y","z"];
n = 2;
load('data/model/uav_'+name(n)+'_model_data.mat')
load('data/model/lqr_'+name(n)+'_params.mat')

% rk = [sine_dt_traj(1,:);zeros(2,length(sine_dt_traj))];

rk = [zeros(1,100),ones(1,4900);zeros(1,5000)];
% rk = sine_traj;
% rk = step_traj;

% x0 = InvSet.V(1,:)';
x0 = zeros(system.nx,1);
Nsims = 1;
N = 80;
xk = zeros(system.nx,N);
xk(:,1) = x0';
uk = zeros(system.nu,N);
ck = zeros(1,N);

reftraj = reshape(rk,1,[]);

setpoint = sdpvar(system.nx,1);

% Model data
A = system.A;
display(A)
B = system.B;

nx = system.nx; % Number of states
nu = system.nu; % Number of inputs

% MPC data
N = 80;
Q = system.x.penalty.weight;
R = system.u.penalty.weight;

Ublocking = [1 1 1 1 1 5 5 5 5 5 10];
Nmpc = sum(Ublocking);
Ncblock = length(Ublocking);
u = sdpvar(repmat(nu,1,Ncblock),ones(1,Ncblock));
x = sdpvar(repmat(nx,1,Nmpc+1),ones(1,Nmpc+1));
r = sdpvar(repmat(nx,1,Nmpc+1),ones(1,Nmpc+1));

Uindex = zeros(size(Ublocking));
for k = 1:Ncblock
    Uindex(k) = sum(Ublocking(1:k));
end
display(A)
constraints = [];
objective = 0;
j = Uindex(1);
for k = 1:Nmpc
    if Uindex(j) < k
        j = j+1;
    end
    objective = objective + (x{k}-r{k})'*Q*(x{k}-r{k}) + u{j}'*R*u{j};
    constraints = [constraints, x{k+1} == A*x{k}+B*u{j}];
    constraints = [constraints, system.u.min <= u{j}; u{j} <= system.u.max, system.x.min<=x{k+1}; x{k+1} <=system.x.max];
    if k == 1
        dt = 0.2;
        A = [1 dt; 0 1];
        B = [0;dt];
        Q = 10*system.x.penalty.weight;
        R = 10*system.u.penalty.weight;
    end
end
objective = objective + (x{Nmpc+1}-r{Nmpc+1})'*Q*(x{Nmpc+1}-r{Nmpc+1});

parameters_in = {x{1},[r{:}]};
solutions_out = {[u{:}], [x{:}]};
options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);  % setup LP-solver
controller = optimizer(constraints, objective,options,parameters_in,solutions_out);
x = x0;

xk = zeros(system.nx,N);
xk(:,1) = x;
uk = zeros(1,N);
time  = zeros(N-1,Nsims);
tic;
for j = 1:Nsims
x = x0;
xk(:,1) = x;
for i = 1:N-1
    i
%     future_r = rk(:,i:i+N);%[10*sin(9*pi*(i:i+N)/N);3/5*pi*cos(9*pi*(i:i+N)/(N/3))];   
    future_r = [rk(:,i),rk(:,i+1:20:i+N)];
    inputs = {x,future_r};
    [solutions,diagnostics] = controller{inputs};    
    U = solutions{1};
    X = solutions{2};
    x = system.A*x + system.B*U(1);
    uk(:,i) = U(1);
    xk(:,i+1) = x;
    time(i,j) = toc;
end
j
end

% save('results/data/mpc_step_blocking_time.mat','xk','uk','rk','time','Nsims','N');

% figure(1);
% stairs(xk');
% xlabel('k');
% ylabel('x_k');
% 
% figure(2);
% stairs(uk);
% xlabel('k');
% ylabel('u_k');
% 
% figure(3);
% stairs(ck);
% xlabel('k');
% ylabel('c_k');

% cd results/scripts