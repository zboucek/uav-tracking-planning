function ut = mpc_block_y(currentx,reftraj)

persistent Controller

tstart = tic;
if isempty(Controller)
    % init the controller with data from files
    load('data/model/uav_y_0.01_model_data.mat','system')

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

%     N = (length(reftraj)/2) -1;
    Ublocking = [1 1 1 1 1 5 5 5 5 5 10];
    Nmpc = sum(Ublocking);
    Ncblock = length(Ublocking);
    u = sdpvar(repmat(system.nu,1,Ncblock),ones(1,Ncblock));
    x = sdpvar(repmat(system.nx,1,Nmpc+1),ones(1,Nmpc+1));
    r = sdpvar(repmat(system.nx,1,Nmpc+1),ones(1,Nmpc+1));
    
    Uindex = zeros(size(Ublocking));
    for k = 1:Ncblock
        Uindex(k) = sum(Ublocking(1:k));
    end
    
    j = Uindex(1);
    constraints = [];
    objective = 0;
    for k = 1:Nmpc
        if Uindex(j) < k
            j = j+1;
        end
        objective = objective + (x{k}-r{k})'*system.x.penalty.weight*(x{k}-r{k}) + u{j}'*system.u.penalty.weight*u{j};
        constraints = [constraints, x{k+1} == system.A*x{k}+system.B*u{j}];
        constraints = [constraints, system.u.min <= u{j}, u{j} <= system.u.max, system.x.min <= x{k+1}, x{k+1} <= system.x.max];
        if k == 1
            load('data/model/uav_y_0.2_model_data.mat','system')
        end
    end
    objective = objective + (x{Nmpc+1}-r{Nmpc+1})'*system.x.penalty.weight*(x{Nmpc+1}-r{Nmpc+1});
    
    options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);
    Controller = optimizer(constraints, objective, options, {x{1},[r{:}]},u{1});
end

reftraj = reshape(reftraj,2,[]);
reftraj = [reftraj(:,1),reftraj(:,2:20:end)];
uout = Controller{{currentx,reftraj}};
t = toc(tstart);
ut = [uout, t];

end
