function [QPsolver,lbg,ubg,N] = makeReferenceGovernor(G,PHI,Hy,Hc,Cset,T,DT,solverOpt)
    %% Generate Reference Governor function
    %  Author: Giovanni Licitra
    % Inputs --------------------------------------------------------------
    %  G   : State matrix  plant+controller
    % PHI  : Input matrix  plant+controller
    % Hy   : Output matrix plant+controller
    % Hc   : Constraint matrix plant+controller
    % Cset : Set constraint
    % T    : Time Horizon
    % DT   : Sample Time
    % Outputs -------------------------------------------------------------
    % QPsolver : QP solver
    % lbg      : lower bound QP formuation
    % ubg      : upper bound QP formuation
    % N        : Prediction Horizon 
    
    global nx nr
    import casadi.*
    %% Model 
    x  = SX.sym('x' ,nx); % plant state + PID state
    rp = SX.sym('rp',nr); % predictive reference
    r  = SX.sym('r' ,nr); % reference desired [trajectory prewiev] 

    % Continuous time dynamics
    xdot = PHI*x + G*rp;
    L    = (Hy*x-r)'*(Hy*x-r);
    f    = Function('f', {x, rp, r}, {xdot, L} ,...
           char('states', 'Predictive_reference','reference'), char('ode', 'Mayer Term'));

    %% ====================================================================
    % Control discretization
    M  = 4;        % runge-kutta 4
    N  = T/DT; % Prediction Horizon

    X0p     = MX.sym('X0', nx);
    U       = MX.sym('U' , nr);
    R       = MX.sym('R' , nr);
    X       = X0p;
    Q       = 0;

    for j=1:M
        [k1, k1_q] = f(X            , U , R);
        [k2, k2_q] = f(X + DT/2 * k1, U , R);
        [k3, k3_q] = f(X + DT/2 * k2, U , R);
        [k4, k4_q] = f(X + DT   * k3, U , R);
        X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4  );
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
    F  = Function('F', {X0p, U , R}, {X, Q});

    %% Start with an empty NLP ================================================
    w      = {};  % solution array
    J      = 0; 
    g      = {};
    lbg    = [];
    ubg    = [];
    r      = {};  % trajectory+x0 used as parameters

    % "Lift" initial conditions
    X0p    = MX.sym('X0', nx);
    r      = {r{:}, X0p};

    % Formulate the NLP
    Xk = X0p;
    for k=0:N-1
        % New NLP variable for the control
        Uk  = MX.sym(['U_' num2str(k)],nr);
        w   = {w{:}, Uk};

        % Integrate till the end of the interval
        Rk  = MX.sym(['RX_' num2str(k)], nr);

        [Xk_end, Jk] = F(Xk, Uk, Rk);
        J = J + Jk;

        % define path constrains
        Constraints = Hc*Xk;
        Y_Torque   = Constraints(1);
        Y_Voltage  = Constraints(2);

        % New NLP variable for state at end of interval
        Xk  = MX.sym(['X_' num2str(k+1)], nx);
        w   = {w{:}, Xk};

        % Add equality and Path constraints
        g   = {g{:}, Xk_end-Xk , Y_Torque , Y_Voltage};
        lbg = [lbg; zeros(nx,1); -Cset];
        ubg = [ubg; zeros(nx,1);  Cset];
        r   = {r{:}, Rk};
    end
    
    % final path constraint
    g   = {g{:}, Y_Torque , Y_Voltage};
    lbg = [lbg; -Cset];
    ubg = [ubg;  Cset];
        
    if strcmp(solverOpt,'ipopt')   
        % Create an NLP solver
        opts                             = struct;
        opts.expand                      = true;
        opts.ipopt.max_iter              = 50;
        opts.ipopt.point_perturbation_radius = 0;
        opts.ipopt.linear_solver         = 'ma27';
        opts.ipopt.hessian_approximation = 'exact';
        opts.jit                         = true;
        opts.jit_options = struct('flags',char('-O3'));

        % Solve the QP via IPOPT
        prob     = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}),'p',vertcat(r{:}));
        QPsolver = nlpsol('solver', 'ipopt', prob,opts);
        disp(['prediction Horizon N = ',num2str(N)])
    elseif strcmp(solverOpt,'qpoases')        
        % Create an NLP solver
        opts                             = struct;
        opts.jit                         = true;
        opts.jit_options = struct('flags',char('-O3'));
        opts.print_time = true;
        opts.sparse = true;

        % Solve the QP via IPOPT
        prob     = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}),'p',vertcat(r{:}));
        
        c = [prob.x;prob.p];
        He = hessian(prob.f,c);
        Ge = substitute(jacobian(prob.f,c),c,0);
        Ae = jacobian(prob.g,c);
        
        He = sparse(DM(He));
        Ge = sparse(DM(Ge));
        Ae = sparse(DM(Ae));
        
        H = He(1:nx,1:nx);
        G = G(1:nx) + 2*He(1:nx,nx:end);
         
        QPsolver = qpsol('solver', 'qpoases', prob,opts);
        disp(['prediction Horizon N = ',num2str(N)])
%         x = casadi.SX.sym('x',size(vertcat(w{:})));
%         HFun = Function('H',{ vertcat(w{:}) },{ jacobian(J,vertcat(w{:})) });
% %         HFun.expand(); % get SX function
%         H = DM(HFun(x));
%         
%         AFun = Function('A',{ vertcat(w{:}) },{ jacobian(vertcat(g{:}),vertcat(w{:})) });
%         AFun.expand();  % get SX function
%         A = DM(AFun(x));
% %          spy(sparse(H)) for e.g. quadprog    
%          
%         qp = struct;
%         qp.h = H.sparsity();
%         qp.a = A.sparsity();
%         QPsolver = conic('QPsolver','qpoases',qp);    
%         
%         opts                     = struct;
%         opts.jit                 = true;
%         opts.linsol_plugin       = 'ma27';
% 
%        prob     = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}),'p',vertcat(r{:}));
%        QPsolver = qpsol('solver', 'sqic', prob,opts);        end
end