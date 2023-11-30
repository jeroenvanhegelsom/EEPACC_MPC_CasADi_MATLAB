function optSol = RunOpt_FBMPC(OPTsettings)
% RunOpt_FBMPC Runs the force-based MPC over the simulation
%
%   Inputs:
%       OPTsettings : struct with settings for the MPC
%
%   Output:
%       optSol      : struct containing the optimal solution, also
%                     including other relevant data

%% Import CasADi

import casadi.*

%% Unpack settings

% optimization settings
W            = OPTsettings.W_FB;
N_hor        = OPTsettings.N_hor;
b_fifthOrder = OPTsettings.b_fifthOrder;
solverToUse  = OPTsettings.solverToUse;
FBuseTaylor  = OPTsettings.FBuseTaylor;
Tvec         = OPTsettings.Tvec;
Mb           = OPTsettings.Mb;
t_sim        = OPTsettings.t_sim;
s_0          = OPTsettings.s_init;
v_0          = OPTsettings.v_init;
a_minus1     = OPTsettings.a_minus1;

% vehicle following
s_tv = OPTsettings.s_tv;
v_tv = OPTsettings.v_tv;

% get vehicle parameters
V = SetVehicleParameters();

% determine number of steps for the simulation
Ts    = Tvec(1);
N_sim = t_sim/Ts;

%% Initialize matrices and create solver

% numbers of variables
if solverToUse == 2
% HPIPM
    n_x = 5; % [s; v; v_prev; Fm_prev; Fb_prev]
else
% qpOASES
    n_x = 2; % [s; v]
end
n_u = 6; % [Fm; Fb; xi_v; xi_h; xi_s; xi_f]
n_x_u = n_x + n_u;

% initialization of matrices: estimated values don't matter
s_est       = ones(N_hor+1,1);
v_est       = ones(N_hor+1,1);
s_tv_est    = ones(N_hor+1,1);
Fm_prev     = 7;
Fb_prev     = 2;
v_prev      = 5;
t_0         = 1;

% Get matrices and vectors for sparsity patterns
[H,c,G,~,~,g_lb,g_ub] = CreateQP_FB(OPTsettings,n_x,n_u,s_0,v_0,s_est,v_est,s_tv_est,t_0,v_prev,a_minus1,Fm_prev,Fb_prev);

% delete zero rows in G
zeroColsG = all(~G,2);
G(zeroColsG,:) = [];

% Fix lengths of bounds on G
g_lb = g_lb(1:size(G,1));
g_ub = g_ub(1:size(G,1));

% Transform to dense formulation if required
if solverToUse == 1

    % Create state space matrices
    A = zeros(N_hor,n_x,n_x);
    B = zeros(N_hor,n_x,n_u);
    D = zeros(N_hor,n_x);
    for k = 1:N_hor
        if FBuseTaylor
            A(k,1:2,1:2) = [1, Tvec(k); 0, (1 - 2*Tvec(k)*V.zeta_a*v_0^2/V.lambda/V.m)];
            D(k,:) = [0; Tvec(k)/V.lambda/V.m*(V.zeta_a*v_0^2)]; % exact value doesn't matter in the initialization
        else
            A(k,1:2,1:2) = [1, Tvec(k); 0, 1];
            D(k,:) = [0; Tvec(k)/V.lambda/V.m*(-V.zeta_a*v_0^2)]; % exact value doesn't matter in the initialization
        end
        B(k,1:2,1:2) = Tvec(k)/V.lambda/V.m*[0, 0; 1, 1];
    end

    [H,~,G,~,~,~,~] = TransformToDenseFormulation(N_hor,A,B,D,H,c,G,g_lb,g_ub,n_x,n_u,s_0,v_0);
end

% create problem structure and solver
if solverToUse == 2
% HPIPM
    % # tot. constraints = {# cols G} - {# state cont. constr.} - {# init. state constr.} - {# final stage constr.} - {# move blocking constr.}
    ng_num = (size(G,1)-(N_hor+1)*n_x - 2 - sum(Mb))/N_hor; 
    optsHPIPM = struct;
    optsHPIPM.N = N_hor;
    optsHPIPM.nx = n_x*ones(N_hor+1,1);
    optsHPIPM.nu = [n_u*ones(N_hor,1);0];
    ng = [ng_num+n_x;ng_num*ones(N_hor-1,1);n_x-3]; % add constraints: {# init. state constr.},{# final stage constr.}
    optsHPIPM.ng = ng + [Mb,0]'; % add move blocking constraints
    optsHPIPM.hpipm.iter_max = OPTsettings.FBMPCmaxIterHPIPM;
        % other optional options
        %     optsHPIPM.hpipm.res_g_max = 1e-10;
        %     optsHPIPM.hpipm.res_b_max = 1e-10;
        %     optsHPIPM.hpipm.res_d_max = 1e-10;
        %     optsHPIPM.hpipm.res_m_max = 1e-10;
        %     optsHPIPM.hpipm.mode = "robust";
        %     optsHPIPM.print_time = true;
        %     optsHPIPM.inf = 1e7;
    optsHPIPM.print_problem = false;
    optsHPIPM.hpipm.warm_start = true;
    optsHPIPM.error_on_fail = false;
    H = sparse(H);
    G = sparse(G);
    prob = struct('h', SX(H).sparsity(), 'a', SX(G).sparsity());
    QPsolver = conic('solver','hpipm',prob,optsHPIPM);
else
% qpOASES
    opts = struct;
    opts.printLevel = 'low'; % options: none, low, medium, high, debug_iter, tabular
    opts.sparse = false;
    opts.error_on_fail = false;
    prob = struct('h', SX(H).sparsity(), 'a', SX(G).sparsity());
    QPsolver = conic('QPsolver','qpoases',prob,opts);
end

%% MPC loop

% initialize arrays
s_opt     = zeros(t_sim/Ts,1);
v_opt     = s_opt;
Fm_opt    = s_opt;
Fb_opt    = s_opt;
xi_v_opt  = s_opt;
xi_h_opt  = s_opt;
xi_s_opt  = s_opt;
xi_f_opt  = s_opt;
E_opt     = s_opt;
a_opt     = s_opt;
DistHor   = s_opt;
cost      = s_opt;
tLoop     = s_opt;
tSolve    = s_opt;
cost_P    = s_opt;
cost_a    = s_opt;
cost_j    = s_opt;
cost_xi_v = s_opt;
cost_xi_h = s_opt;
cost_xi_s = s_opt;
cost_xi_f = s_opt;

% simulation time in the first step of each MPC loop (s)
t_0  = 0;                

% Loop over MPC steps
for kk = 0:N_sim 
    k = kk+1;
    tLoopTic = tic();
    %% Measurements and estimations

    if kk == 0
        % current state is initial state at t0
        s_measured = s_0;
        v_measured = v_0;
        
        % Previous solution is unknown (assumption)
        s_opt_prev_sol = zeros(N_hor+1,1);
        v_opt_prev_sol = zeros(N_hor+1,1);

        % Previous controls (assumption)
        Fm_prev = 0;
        Fb_prev = 0;

        % initial target vehicle measurement
        s_tv_measured = s_tv(k);
        v_tv_measured = 0;
        v_tv_prev     = v_tv_measured;
        a_tv_prev     = (v_tv_measured - v_tv_prev)/Ts;
    else
        % Previous states and controls
        s_prev  = s_opt(k-1);
        v_prev  = v_opt(k-1);
        Fm_prev = Fm_opt(k-1);
        Fb_prev = Fb_opt(k-1);

        % Get measurements from simulated nonlinear plant (4 RK4 step integrator)
        [s_measured,v_measured] = RunPlantModel([s_prev,v_prev],[Fm_prev,Fb_prev],OPTsettings);

        % Estimate of the previous acceleration
        a_minus1 = (v_measured-v_prev)/Ts;

        % Headway distance measurement and calculations
        s_tv_measured = s_tv(k);
        v_tv_prev     = v_tv_measured;
        v_tv_measured = v_tv(k);
        a_tv_prev     = (v_tv_measured - v_tv_prev)/Ts;
    end

    % estimates of the ego vehicle's trajectory
    [s_est, v_est] = EstimateVehicleTrajectory(OPTsettings,0,s_measured,v_measured,a_minus1,s_opt_prev_sol,v_opt_prev_sol);

    % Estimate the target vehicle's distances
    [s_tv_est, v_tv_est] = EstimateVehicleTrajectory(OPTsettings,1,s_tv_measured,v_tv_measured,a_tv_prev);

    % save horizon distance
    DistHor(kk+1) = s_est(end) - s_measured;

    %% Formulate the the QP
    
    [H,c,G,z_lb,z_ub,g_lb,g_ub,theta_est] = CreateQP_FB(OPTsettings,n_x,n_u,s_measured,v_measured,s_est,v_est,s_tv_est,t_0,v_prev,a_minus1,Fm_prev,Fb_prev);

    %% Postprocessing
    
    if solverToUse == 2
        % HPIPM: change inf values in bounds to 1e7
        for i = 1:length(g_lb)
            if abs(g_lb(i)) > 1e7
                g_lb(i) = sign(g_lb(i))*1e7;
            end
            if abs(g_ub(i)) > 1e7
                g_ub(i) = sign(g_ub(i))*1e7;
            end
        end
        for i = 1:length(z_lb)
            if abs(z_lb(i)) > 1e7
                z_lb(i) = sign(z_lb(i))*1e7;
            end
            if abs(z_ub(i)) > 1e7
                z_ub(i) = sign(z_ub(i))*1e7;
            end
        end
    end

    % Delete zero rows in G
    G(zeroColsG,:) = [];

    % Fix lengths of bounds on G
    g_lb = g_lb(1:size(G,1));
    g_ub = g_ub(1:size(G,1));

    % Get estimate for state space matrices
    for i = 1:N_hor

        if FBuseTaylor
            % if Taylor linearization is used, A_k and D_k change in each iteration
            A(k,1:2,1:2) = [1, Tvec(i); 
                            0, (1 - 2*Tvec(i)*V.zeta_a*v_est(i)/V.lambda/V.m)];
            D(k,:) = [0; Tvec(i)/V.lambda/V.m*(V.zeta_a*v_est(i)^2 - ...
                     V.m*V.g*(V.c_r*cos(theta_est(i)) + sin(theta_est(i))) )];
        else
            D(i,:) = [0; Tvec(i)/V.lambda/V.m*(-V.zeta_a*v_est(i)^2 - ...
                     V.m*V.g*(V.c_r*cos(theta_est(i)) + sin(theta_est(i))) )];
        end
    end

    % Transform optimization matrices to dense versions (if necessary)
    if solverToUse == 1
        % dense qpOASES
        [H,c,G,g_lb,g_ub,Psi,d] = TransformToDenseFormulation(N_hor,A,B,D,H,c,G,g_lb,g_ub,n_x,n_u,s_measured,v_measured);
    end

    %% Solve MPC and extract states and controls

    % solve the problem and extract the optimal solution
    tSolveTic = tic();
    if solverToUse == 2
    % HPIPM
        H = sparse(H);
        G = sparse(G);
        sol = QPsolver('h',H,'g',c,'a',G,'lbx',z_lb,'ubx',z_ub,'lba',g_lb,'uba',g_ub);
    else
    % qpOASES
        sol = QPsolver('h',H,'g',c,'a',G,'lbx',z_lb,'ubx',z_ub,'lba',g_lb,'uba',g_ub);
    end
    tSolve(k) = toc(tSolveTic);
    optSol.exitMessage(k) = ~QPsolver.stats().success;
    optSol.solverTime(k) = QPsolver.stats().t_proc_solver;
    z_opt = full(sol.x);

    % Transform dense z to sparse z to get the states as well
    if solverToUse == 1
        z_opt = Psi*z_opt+d;
    end

    % extract optimal state and control variables
    s_opt(k) = z_opt(1);
    v_opt(k) = z_opt(2);
    if solverToUse == 2 
    % HPIPM
        Fm_opt(k)   = z_opt(6);    
        Fb_opt(k)   = z_opt(7);  
        xi_v_opt(k) = z_opt(8);
        xi_h_opt(k) = z_opt(9);
        xi_s_opt(k) = z_opt(10);
        xi_f_opt(k) = z_opt(11);
    else 
    % qpOASES
        Fm_opt(k)   = z_opt(3);    
        Fb_opt(k)   = z_opt(4);   
        xi_v_opt(k) = z_opt(5);
        xi_h_opt(k) = z_opt(6);
        xi_s_opt(k) = z_opt(7);
        xi_f_opt(k) = z_opt(8);
    end
    
    % save previous solution's trajectory for approximations in the next step
    s_opt_prev_sol = z_opt(1:n_x_u:end);
    v_opt_prev_sol = z_opt(2:n_x_u:end);

    % Calculate other optimal quantities
    if kk > 0
        a_opt(k) = (v_opt(k)-v_opt(k-1))/Ts;
    end
    cost(k)  = full(sol.cost);
    tLoop(k) = toc(tLoopTic);
    t_0 = t_0 + Ts;

    if OPTsettings.createGifs
        %% plot a frame for the animation
        if kk == 0
            [EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot] = InitiateAnimation_MPC(k,s_opt,v_opt,s_opt_prev_sol,v_opt_prev_sol,s_tv_est,v_tv_est,OPTsettings,'FBMPC');
        else
            PlotAnimFrame_MPC(k,s_opt,v_opt,s_opt_prev_sol,v_opt_prev_sol,s_tv_est,v_tv_est,EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot)
        end
    end
end

% Calculate other optimal quantities
rpm_opt = (30/pi)*v_opt*V.phi;
Tm_opt  = Fm_opt./V.phi./(V.eta_TF.^sign(Fm_opt));
P_opt   = GetMotorPower_FifthOrderSurface(Fm_opt,rpm_opt,b_fifthOrder);
for k = 1:length(P_opt)
    E_opt(k) = Ts*sum(P_opt(1:k));
end

%% prepare output struct

% optimal jerk
j_opt = diff(a_opt)/Ts;

% optimized variables
optSol.s_opt    = s_opt;
optSol.v_opt    = v_opt;
optSol.Fm_opt   = Fm_opt;
optSol.Fb_opt   = Fb_opt;
optSol.xi_v_opt = xi_v_opt;
optSol.xi_h_opt = xi_h_opt;
optSol.xi_s_opt = xi_s_opt;
optSol.xi_f_opt = xi_f_opt;

% derived from optimal variables
optSol.P_opt    = P_opt;
optSol.E_opt    = E_opt;
optSol.a_opt    = a_opt;
optSol.j_opt    = j_opt;
optSol.Tm_opt   = Tm_opt;
optSol.rpm_opt  = rpm_opt;

optSol.tLoop    = tLoop;
optSol.tSolve   = tSolve;

% MPC data
optSol.H        = H;
optSol.G        = G;
optSol.DistHor  = DistHor;

% extract weights
w_P = W(1);
w_a = W(2);
w_j = W(3);
w_v = W(4);
w_h = W(5);
w_s = W(6);
w_f = W(7);

% costs over the horizon
for k = 1:N_sim
    cost_P(k)    = w_P*sum(P_opt(1:k).^2);
    cost_a(k)    = w_a*sum(a_opt(1:k).^2);
    cost_j(k)    = w_j*sum(j_opt(1:k).^2);
    cost_xi_v(k) = w_v*sum(xi_v_opt(1:k));
    cost_xi_h(k) = w_h*sum(xi_h_opt(1:k));
    cost_xi_s(k) = w_s*sum(xi_s_opt(1:k));
    cost_xi_f(k) = w_f*sum(xi_f_opt(1:k));
end
optSol.cost_P    = cost_P;
optSol.cost_a    = cost_a;
optSol.cost_j    = cost_j;
optSol.cost_xi_v = cost_xi_v;
optSol.cost_xi_h = cost_xi_h;
optSol.cost_xi_s = cost_xi_s;
optSol.cost_xi_f = cost_xi_f;
end