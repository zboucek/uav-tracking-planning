function ut = mpc_z(currentx,reftraj)

persistent Controller

tstart = tic;
if isempty(Controller)
    % init the controller with data from files
    load('data/model/uav_z_0.01_model_data.mat')

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    N = (length(reftraj)/2) -1;
    u = sdpvar(repmat(system.nu,1,N),ones(1,N));
    x = sdpvar(repmat(system.nx,1,N+1),ones(1,N+1));
    r = sdpvar(repmat(system.nx,1,N+1),ones(1,N+1));

    constraints = [];
    objective = 0;
    for k = 1:N
        objective = objective + (x{k}-r{k})'*system.x.penalty.weight*(x{k}-r{k}) + u{k}'*system.u.penalty.weight*u{k};
        constraints = [constraints, x{k+1} == system.A*x{k}+system.B*u{k}];
        constraints = [constraints, system.u.min <= u{k}, u{k} <= system.u.max, system.x.min <= x{k+1}, x{k+1} <= system.x.max];
    end
    objective = objective + (x{N+1}-r{N+1})'*system.x.penalty.weight*(x{N+1}-r{N+1});
    
    options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);
    Controller = optimizer(constraints, objective, options, {x{1},[r{:}]},u{1});
end

reftraj = reshape(reftraj,2,[]);
uout = Controller{{currentx,reftraj}};
t = toc(tstart);
ut = [uout, t];

end
