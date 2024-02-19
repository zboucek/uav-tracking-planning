function [P] = invLQSets(varargin)
% INVLQSET search an invariant controlled set for LQ controllers
if nargin < 1
    P = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    sys = varargin{1}.copy;
    K = sys.LQRGain;        % LQ controler for system sys
elseif nargin == 2
    sys = varargin{1}.copy;
    K = varargin{2};
else
    P = NaN;
    warning('wrong input!')
    return;
end

% transform conditions to matrix inequavility representation
Fx = [eye(sys.nx);-eye(sys.nx)];
gx = [sys.x.max; -sys.x.min];
Fu = [eye(sys.nu);-eye(sys.nu)];
gu = [sys.u.max; -sys.u.min];

[~, ~, o] = size(K);

P = Polyhedron.empty(o,0);
for i = 1:o
    Ac = sys.A + sys.B*K(:,:,i);   % closed loop dynamics

    % LMI conditions for closed loop system
    Fc = [Fx; Fu*K(:,:,i)];
    gc = [gx; gu];

    % initialize algorithm for invariant controlled set
    X0 = Polyhedron(Fc,gc);
    X0.minHRep;

    while 1
        Ptemp = Polyhedron([X0.A;X0.A*Ac],[X0.b;X0.b]); % next iteration
        % check if set is found
        if Ptemp == X0
            break;
        else
            % eliminate redundant conditions from half-space representation
            if ~Ptemp.irredundantHRep
                Ptemp.minHRep;
            end
            % set new polyhedron and go to start
            X0 = Ptemp.copy;
        end
    end
    P(i) = Ptemp.copy;
end

end

