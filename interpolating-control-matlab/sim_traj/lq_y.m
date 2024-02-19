function uout = lq_y(currentx,reftraj)

persistent LQ1

if isempty(LQ1)
    % init the controller with data from files
    load('data/model/lqr_y_0.01_params.mat')
end

uout = LQ1.K*currentx + LQ1.L*(reftraj);

end