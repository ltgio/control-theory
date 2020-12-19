clc;
% clear all;
close all;

import casadi.*

% Environment
g = 9.81;     % [N/kg]

% platform charateristics
ma = 0.5;           % Mass of platform [kg]
mb = 0.1;           % Mass of end-effector [kg] 
L  = 0.25;          % Reference length [m]
L_pendulum = 0.20;  % [m]

I_max = 0.1 * L^2; %Inertia of a point mass at a distance L
I_ref = I_max/5;   

I = diag([I_ref/2,I_ref/2,I_ref]); % [N.m^2]

% propellor position
rotors_pos = [L, 0, 0;0, L, 0; -L, 0, 0;0, -L, 0]'; % rotor positions
rotors_N = 4;

% System states
pa  = SX.sym('pa',3);  % platform position
dpa = SX.sym('dpa',3); % platform velocity wrt inertial

pb  = SX.sym('pb',3);  % end-effector position
dpb = SX.sym('dpb',3); % end-effector wrt inertial

R   = SX.sym('R',3,3);   % Rotation matrix from body to inertial
w   = SX.sym('w',3);     % Angular rate, expressed in body
dw  = SX.sym('dw',3);   % Angular acceleration, expressed in body

% System controls
r   = SX.sym('r',rotors_N); % rotor action [N]

% Lagrange mechanics for the translation part
q   = [pa;pb];
dq  = [dpa;dpb];
ddq = SX.sym('ddq',size(q,1),1);

% Potential energy
E_pot = ma*g*pa(3)+mb*g*pb(3);

% Kinetic energy
E_kin = 0.5*ma*(dpa'*dpa)+0.5*mb*(dpb'*dpb);

% Pendulum constraint
e   = pa-pb;
c   = 0.5*(e'*e-L_pendulum^2);
% Time derrivative of constraint
dc  = jtimes(c,q,dq);
% Second time derrivative of constraint
ddc = jtimes(dc,[q;dq],[dq;ddq]);

lam = SX.sym('lam');

% Lagrange function for translation part
Lag = E_kin - E_pot - dot(lam.T,c);

% Force-less Lagrange equations for translation part
eq = jtimes(gradient(Lag,dq),[q;dq],[dq;ddq]) - gradient(Lag,q);

F_platform = 0;
T_platform = 0;

% Aerodynamics
for i=1:rotors_N
    F_aero_body = r(i)*[0;0;1];
    F_platform  = F_platform + R*F_aero_body;
    T_platform  = T_platform + cross(rotors_pos(:,i),F_aero_body);
end

% Residual for translation (from Lagrange)
res_transl = eq - [F_platform;zeros(3,1)];
% Residual for rotation (Euler equations)
res_rot = I*dw+cross(w,I*w)-T_platform;

dae_x = struct('q',q,'dq',dq,'R',R,'w',w);
dae_z = struct('lam',lam,'ddq',ddq,'dw',dw);
dae   = struct;
dae.x = casadi_struct2vec(dae_x);
dae.z = casadi_struct2vec(dae_z);
dae.p = r;
dae.ode = casadi_vec(dae_x,'q',dq,'dq',ddq,'R',R*skew(w),'w',dw);

dae.alg = [res_transl;
           res_rot   ;
           ddc+dc+c ];

nx = size(dae.x,1);
nu = size(dae.p,1);
nz = size(dae.z,1);

% Nominal force per rotor needed to hold quadcopter stationary
r_nom = (ma+mb)*g/rotors_N;

x0_guess = casadi_vec(dae_x,'q',[0;0;0;0;0;L_pendulum],'R',eye(3));
u_guess  = [r_nom;r_nom;r_nom;r_nom];
z_guess  = casadi_vec(dae_z,'lam',-mb*g/L_pendulum);

daefun = Function('daefun',dae,char('x','z','p'),char('ode','alg'));
celldisp(daefun.call({x0_guess,z_guess,u_guess}))

T = 1.0;
N = 14; % Caution; mumps linear solver fails when too large

% Degree of interpolating polynomial
d = 4;

% Get collocation points
tau_root = [0,num2cell(collocation_points(d, 'radau'))];

collfun  = simpleColl(dae,tau_root,T/N);
collfun  = collfun.expand();

Xs  = {};
XCs = {};
Zs  = {};
Us  = {};

for i     = 1:N
   Xs{i}  = MX.sym(['X_' num2str(i)] ,nx  );
   XCs{i} = MX.sym(['XC_' num2str(i)],nx,d);
   Zs{i}  = MX.sym(['Z_' num2str(i)] ,nz,d);
   Us{i}  = MX.sym(['U_' num2str(i)] ,nu  );
end

V_block = struct();
V_block.X  = Sparsity.dense(nx,1);
V_block.XC = Sparsity.dense(nx,d);
V_block.Z  = Sparsity.dense(nz,d);
V_block.U  = Sparsity.dense(nu,1);

Rc           = nonzeros(triu(R'*R-eye(3)));
invariants   = Function('invariants',{dae.x},{[c;dc;vertcat(Rc{:})]});
J_invariants = invariants.jacobian();

thetas = linspace(0,2*pi,N+1);
r0     = 0.0001;

x0_list = {};
for k=1:N
  x0_guess = casadi_vec(dae_x,'q',[r0*cos(thetas(k));r0*sin(thetas(k));0;r0*cos(thetas(k));r0*sin(thetas(k));L_pendulum],'R',eye(3));
  x0_list = {x0_list{:} repmat(x0_guess,d+1,1),repmat(z_guess,d,1),u_guess};
end

x0 = vertcat(x0_list{:});

for r0=[0.0001 0.001 0.01 0.05 0.1]

    % Simple bounds on states
    lbx = {};
    ubx = {};

    % List of constraints
    g   = {};

    % List of all decision variables (determines ordering)
    V   = {};
    for k=1:N
      % Add decision variables
      V = {V{:} casadi_vec(V_block,'X',Xs{k},'XC',XCs{k},'Z',Zs{k},'U',Us{k})};

      if k==1
        % Bounds at t=0
        q_lb  = [r0*cos(thetas(k));r0*sin(thetas(k));-inf;r0*cos(thetas(k));r0*sin(thetas(k));L_pendulum];
        q_ub  = [r0*cos(thetas(k));r0*sin(thetas(k));inf;r0*cos(thetas(k));r0*sin(thetas(k));L_pendulum];
        dq_lb = [-inf;-inf;-inf;0;0;0];
        dq_ub = [inf;inf;inf;0;0;0];
        x_lb = casadi_vec(dae_x,-inf,'q',q_lb,'dq',dq_lb);
        x_ub = casadi_vec(dae_x,inf, 'q',q_ub,'dq',dq_ub);
        lbx = {lbx{:} casadi_vec(V_block,-inf,'X',x_lb)};
        ubx = {ubx{:} casadi_vec(V_block,inf, 'X',x_ub)};
      elseif k==N/2
        q_lb  = [-inf;-inf;-inf;r0*cos(thetas(k));r0*sin(thetas(k));L_pendulum];
        q_ub  = [inf;inf;inf;r0*cos(thetas(k));r0*sin(thetas(k));L_pendulum];
        dq_lb = [-inf;-inf;-inf;-inf;-inf;-inf];
        dq_ub = [inf;inf;inf;inf;inf;inf];
        x_lb = casadi_vec(dae_x,-inf,'q',q_lb,'dq',dq_lb);
        x_ub = casadi_vec(dae_x,inf, 'q',q_ub,'dq',dq_ub);
        lbx = {lbx{:} casadi_vec(V_block,-inf,'X',x_lb)};
        ubx = {ubx{:} casadi_vec(V_block,inf, 'X',x_ub)};  
      else
        % Bounds for other t
        lbx = {lbx{:} casadi_vec(V_block,-inf)};
        ubx = {ubx{:} casadi_vec(V_block,inf)};
      end
      % Obtain collocation expressions
      coll_out = collfun.call({Xs{k},XCs{k},Zs{k},Us{k}});

      g = {g{:} coll_out{2}};         % collocation constraints
      if k<N
        g = {g{:} Xs{k+1}-coll_out{1}}; % gap closing
      else
        out = J_invariants.call({Xs{1}});
        Z = nullspace(out{1});
        g = {g{:} Z'*(Xs{1}-coll_out{1})};
      end
    end

    % Construct regularisation
    reg = 0;
    for k=1:N
      x = Xs{k};
      xstruct = casadi_vec2struct(dae_x,x);
      reg = reg + sum1(sum2((xstruct.R-DM.eye(3)).^2));

      reg = reg + sum1(sum2((xstruct.q(1:3)-xstruct.q(4:6)-[0;0;L_pendulum]).^2));
      reg = reg + sum1(sum2(xstruct.w.^2));
      reg = reg + sum1(sum2(xstruct.dq.^2));
    end

    for k=1:N
      u = Us{k};
      reg = reg + 100*sum1(sum2((u-r_nom).^2));
    end

    e = vertcat(Us{:});

    inv_out = invariants.call({Xs{1}});
    nlp = struct('x',vertcat(V{:}), 'f',(e'*e)+reg, 'g', [inv_out{1};vertcat(g{:})]);

    nlpfun = Function('nlp',nlp,char('x','p'),char('f','g'));
    
    opts = struct;
    opts.ipopt.linear_solver = 'ma57';
    solver = nlpsol('solver','ipopt',nlp,opts);

    args = struct;
    args.x0 = x0;
    args.lbx = vertcat(lbx{:});
    args.ubx = vertcat(ubx{:});
    args.lbg = 0;
    args.ubg = 0;

    res = solver.call(args);

    dim = size(casadi_struct2vec(V_block));
    res_split = vertsplit(res.x,dim(1));

    res_U = {};
    for r=res_split
        rs = casadi_vec2struct(V_block,r{1});
        res_U = {res_U{:} rs.U};
    end

    res_X = {};
    for r=res_split
        rs = casadi_vec2struct(V_block,r{1});
        res_X = {res_X{:} rs.X};
    end

    sol_q = [];
    invariants_err = [];
    for x=res_X
        out = invariants.call(x);
        invariants_err = [invariants_err full(out{1})];
        
        sol = casadi_vec2struct(dae_x,x{1});

        sol_q = [sol_q full(sol.q)];
    end
    
    figure(1);
    subplot(2,2,1);
    plot(T/N*[0:N-1],sol_q')
    legend('pa_x', 'pa_y', 'pa_z', 'pb_x', 'pb_y', 'pb_z')
    xlabel('time(s)')
    title('States')
    
    subplot(2,2,3);
    plot(T/N*[0:N-1],full([res_U{:}])')
    legend('U1', 'U2', 'U3', 'U4')
    xlabel('time(s)')
    title('Controls')
    
    subplot(2,2,[2 4]);
    plot(T/N*[0:N-1],max(abs(invariants_err),[],1))
    xlabel('time(s)')
    title('Invariants')
    
    x0 = res.x;
end


%% 3D visualization:
% Fontsize = 15;
% L = 0.05;
% figure(2);
% clf
% for k = 0:N-1
%     cur_time = k*T/N;
%     states = casadi_vec2struct(dae_x,res_X{k+1});
%     
%     whitebg([1.0 1.0 1.0])
%     set(gcf,'Color',[1 1 1])
%     
%     q = full(states.q);
%     x = q(1);
%     y = q(2);
%     z = q(3);
%     if length(q) > 3
%         x2 = q(4);
%         y2 = q(5);
%         z2 = q(6);
%     end
%     
%     R = full(states.R);
%     
%     widths = [1.5 1.5 0.75];
%     ls = [1 1 0.3];
% %     line([0 1],[0 0],[0 0],'color','k');hold on
% %     line([0 0],[0 1],[0 0],'color','k');hold on
% %     line([0 0],[0 0],[0 0.8],'color','k');hold on
%     for vec = 1:3
%         L2 = ls(vec)*L;
%         line([x-L2*R(vec,1) x+L2*R(vec,1)],[y-L2*R(vec,2) y+L2*R(vec,2)],[z-L2*R(vec,3) z+L2*R(vec,3)],'color','k','linewidth',widths(vec));hold on
%     end
%     plot3(x,y,z,'rs','MarkerFaceColor','r','MarkerSize',8)
%     if length(q) > 3
%         plot3(x2,y2,z2,'bo','MarkerFaceColor','b','MarkerSize',6)
%         line([x x2],[y y2],[z z2],'color','b','linewidth',1.5);
%     end
%     axis([-0.15 0.15 -0.15 0.15 -0 0.6])
%     view(30,20)
%     plot3(sol_q(1,:),sol_q(2,:),sol_q(3,:),'--g','linewidth',2)
%     plot3(r0*cos(thetas(1)),r0*sin(thetas(1)),L_pendulum,'+r','MarkerSize',10)
%     plot3(r0*cos(thetas(N/2)),r0*sin(thetas(N/2)),L_pendulum,'+r','MarkerSize',10)
%     xlabel('x','FontSize',Fontsize);
%     ylabel('y','FontSize',Fontsize);
%     zlabel('z','FontSize',Fontsize);
%     grid ON
%     drawnow
%     
%     if k == 0
%         text(-0.3,1,0.5,['Press any key to start the visualization..'],'FontSize',15);
%         drawnow
%         pause
%     else
%         pause(0.1);
%     end
% end

%%
options = struct;
options.tf = T/N/10;
options.rootfinder = 'newton';
options.number_of_finite_elements = 1;
options.interpolation_order = 4;
options.collocation_scheme='radau';
options.rootfinder_options = struct('abstol',1e-9);

intg = integrator('intg','collocation',dae,options);

x = full(res_X{1});
xfine = [];
for k=1:N
    u = res_U{k};
    for i=1:10
       args = struct;
       args.x0 = x;
       args.p  = u;
       out = intg.call(args);
       x = out.xf;
       xfine = [xfine x];
    end
end

xfine = full(xfine);
figure;
subplot(1,2,1);
plot(xfine(1,:),xfine(2,:));
xlabel('pa_x');
ylabel('pa_y');

subplot(1,2,2);
plot(xfine(4,:),xfine(5,:));
xlabel('pb_x');
ylabel('pb_y');

%%
options = struct;
options.tf = T/N;
% options.implicit_solver = 'newton';
options.number_of_finite_elements = 1;
options.interpolation_order = 4;
options.collocation_scheme='radau';
% options.implicit_solver_options = struct('abstol',1e-9);

intg = integrator('intg','collocation',dae,options);

x0 = MX.sym('x',nx);
x= x0;
for k=1:N
    u = res_U{k};

    args = struct;
    args.x0 = x;
    args.p  = u;
    out = intg.call(args);
    x = out.xf;
end

% F = Function('fun',{x0},{x});
% J = Function('Jfun',{x0},{jacobian(F,x0)})
% 
% %ssJac = Function('ssJac',{[q;dq;lam],u},{jacobian(ssEq,[lam;u])});
% 
% J = F.jacobian();
% 
% out = J.call({res_X{1}});
% 
% residual = out{2}-res_X{1};
% M = full(out{1});
% 
% % System has many undampned modes <-> eigenvalues on the unit circle
% % In addition, there are two unstable modes from the inverted pendulum
% abs(eig(M))


