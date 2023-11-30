function [H,c,G,z_lb,z_ub,g_lb,g_ub] = CreateQP_AB(OPTsettings,n_x,n_u,s_0,v_0,s_est,v_est,s_tv_est,t_0,a_minus1)
% CreateQP_AB Creates the acceleration-based QP for qpOASES and HPIPM, i.e. it builds the matrices and vectors that define the optimization problem
%
%   Inputs:
%       OPTsettings : struct with settings for the optimization
%       n_x         : number of states in each time step
%       n_u         : number of controls in each time step
%       s_0         : initial travel distance for the QP
%       v_0         : initial velocity for the QP
%       s_est       : estimated travel distance over the horizon
%       v_est       : estimated velocity over the horizon
%       s_tv_est    : estimated travel distance of the target vehicle over the horizon
%       t_0         : initial time for the QP
%       a_minus1    : acceleration in the previous time step
%
%   Outputs:
%       H    : Hessian matrix (matrix of quadratic term in objective)
%       c    : vector defining the linear terms in the objective
%       G    : matrix defining the linear constraints
%       z_lb : vector with lower bounds of all optimization variables
%       z_ub : vector with upper bounds of all optimization variables
%       g_lb : vector with lower bounds of the constraints defined in G
%       g_ub : vector with upper bounds of the constraints defined in G

%% Extract settings

solverToUse = OPTsettings.solverToUse;
Tvec        = OPTsettings.Tvec;
W           = OPTsettings.W_AB;
tau_min     = OPTsettings.tau_min;
h_min       = OPTsettings.h_min;
N           = OPTsettings.N_hor;
s_goal      = OPTsettings.s_goal;
Mb          = OPTsettings.Mb;

% get vehicle parameter struct
V = SetVehicleParameters();

% extract weights
w_a = W(1);
w_j = W(2);
w_v = W(3);
w_h = W(4);
w_s = W(5);
w_f = W(6);

%% Initialize

% Estimate bounds on velocity from the route and comfort limits
[~,v_lim_max,v_stop_max,v_TL_max,v_curv_max,a_min_est,a_max_est,...
    j_min_est,j_max_est] = EstimateRouteAndComfortBounds(OPTsettings, 0, s_est, v_est, t_0, N);

% Estimate minimum velocity to use as an incentive to trave
v_minIncentive = min(cat(2,v_lim_max,v_curv_max),[],2);

% initialize vectors and matrices
n_c   = n_x + N*30;
n_x_u = n_x + n_u;
n_z   = n_x_u*N;
H = zeros(n_z+n_x,n_z+n_x);
c = zeros(n_z+n_x,1);
G = zeros(n_c,n_z+n_x);
g_lb = zeros(n_c,1);
g_ub = g_lb;

% bounds on the state and control variables
s_min       = 0;                        % minimum distance (= pos relative to origin)  (m)
s_max       = s_goal;                   % maximum distance                             (m)
v_min       = 0;                        % minimum velocity                             (m/s)
v_max       = V.v_max;                  % maximum velocity                             (m/s)
a_min       = -8;                       % minimum acceleration                         (m/s^2)
a_max       = 8;                        % maximum acceleration                         (m/s^2)
slack_min   = 0;
slack_max   = inf;

%% Set bounds on z

if solverToUse == 2
% HPIPM
    % Minimum and maximum bounds on states and controls
    z_lb(1:n_x_u:(n_x_u*N+n_x)) = s_min;       % s
    z_ub(1:n_x_u:(n_x_u*N+n_x)) = s_max;
    z_lb(2:n_x_u:(n_x_u*N+n_x)) = v_min;       % v
    z_ub(2:n_x_u:(n_x_u*N+n_x)) = v_max;
    z_lb(3:n_x_u:(n_x_u*N+n_x)) = a_min;       % a (prev)
    z_ub(3:n_x_u:(n_x_u*N+n_x)) = a_max;
    z_lb(4:n_x_u:(n_x_u*N+n_x)) = a_min;       % a (curr)
    z_ub(4:n_x_u:(n_x_u*N+n_x)) = a_max;
    z_lb(5:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_v
    z_ub(5:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(6:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_h
    z_ub(6:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(7:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_s
    z_ub(7:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(8:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_f
    z_ub(8:n_x_u:(n_x_u*N+n_x)) = slack_max;
else
% qpOASES:
    % set lower and upper bounds of z (all infinite)
    if solverToUse == 1
        % dense
        z_lb = -inf*ones(N*n_u,1);
        z_ub = -z_lb;
    else
        % sparse
        z_lb = -inf*ones(N*(n_x+n_u)+n_x,1);
        z_ub = -z_lb;
    end
end

%% Set objective function and constraints (G and its bounds)

if solverToUse == 2
% HPIPM
    % indices of z (mod n_x_u + 1):
    scurr   = 1;        % distance                           s 
    vcurr   = 2;        % velocity                           v
    aprev   = 3;        % previous acceleration              a_prev
    acurr   = 4;        % acceleration                       a
    xi_v    = 5;        % velocity incentive slack variable  xi_v
    xi_h    = 6;        % headway slack variable             xi_h
    xi_s    = 7;        % safety-critical feasibility slack  xi_s
    xi_f    = 8;        % feasibility slack variable         xi_f
else
% qpOASES
    % indices of z (mod n_x_u + 1):   
    scurr   = 1;        % distance                           s 
    vcurr   = 2;        % velocity                           v
    aprev   = 3-n_x_u;  % previous acceleration              a_prev
    acurr   = 3;        % acceleration                       a_curr
    xi_v    = 4;        % velocity incentive slack variable  xi_v
    xi_h    = 5;        % headway slack variable             xi_h
    xi_s    = 6;        % safety-critical feasibility slack  xi_s
    xi_f    = 7;        % feasibility slack variable         xi_f
end

cstrInd = 1;
for kk = 0:N-1

    % k is the index for the estimates (1 to N), kk is the true time step index (0 to N-1)
    k = kk+1;

    % Get the time step at time k
    T = Tvec(k);

    % offset to simplify notation
    o = kk*n_x_u;

    %% === Objective function ===

    % objective function: acceleration term
    ii = o+acurr;
    H(ii,ii) = H(ii,ii) + 2*w_a;
    
    % objective function: jerk term
    if kk == 0
        ii = o+acurr;
        H(ii,ii) = H(ii,ii) + 2*w_j/T^2;
        c(ii) = c(ii) - 2*w_j/T*a_minus1;
    else
        ii = o + [acurr,aprev];
        H(ii,ii) = H(ii,ii) + 2*w_j/T^2*[1, -1; -1, 1];
    end

    % objective function: slack variables
    c(o+xi_v) = c(o+xi_v) + w_v;
    c(o+xi_h) = c(o+xi_h) + 1e2*w_h;
    H(o+xi_h,o+xi_h) = H(o+xi_h,o+xi_h) + 2*w_h;
    c(o+xi_s) = c(o+xi_s) + w_s;
    c(o+xi_f) = c(o+xi_f) + w_f;   

    %% === Constraints ===

    if solverToUse == 2
    % HPIPM

        % continuity of the state variables
        ii = o+[scurr,vcurr,acurr,n_x_u+[scurr,vcurr,aprev]];
        G(cstrInd,ii) = [1, T, .5*T^2, -1,  0,  0]; % continuity of distance
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
        G(cstrInd,ii) = [0,  1,  T,  0, -1,  0]; % continuity of velocity
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
        G(cstrInd,ii) = [0,  0,   1,  0,  0, -1]; % previous acceleration
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;

        % set initial states
        if kk == 0
            G(cstrInd,o+scurr) = 1; % distance
            g_lb(cstrInd) = s_0;
            g_ub(cstrInd) = s_0;
                cstrInd = cstrInd + 1;
            G(cstrInd,o+vcurr) = 1; % velocity
            g_lb(cstrInd) = v_0;
            g_ub(cstrInd) = v_0;
                cstrInd = cstrInd + 1;
            G(cstrInd,o+aprev) = 1; % acceleration (prev)
            g_lb(cstrInd) = a_minus1;
            g_ub(cstrInd) = a_minus1;
                cstrInd = cstrInd + 1;
        end
    else
    % qpOASES

        if solverToUse == 0 % sparse qpOASES: add dynamics

            % continuity of the state variables
            ii = o+[scurr,vcurr,acurr,n_x_u+[scurr,vcurr]];
            G(cstrInd,ii) = [1, T,  T^2, -1,  0]; % continuity of distance
            g_lb(cstrInd) = 0;
            g_ub(cstrInd) = 0;
                cstrInd = cstrInd + 1;
            G(cstrInd,ii) = [0,  1, T,  0, -1]; % continuity of velocity
            g_lb(cstrInd) = 0;
            g_ub(cstrInd) = 0;
                cstrInd = cstrInd + 1;
                
            % set initial states (s and v)
            if kk == 0
                G(cstrInd,o+scurr) = 1; % distance
                g_lb(cstrInd) = s_0;
                g_ub(cstrInd) = s_0;
                    cstrInd = cstrInd + 1;
                G(cstrInd,o+vcurr) = 1; % velocity
                g_lb(cstrInd) = v_0;
                g_ub(cstrInd) = v_0;
                    cstrInd = cstrInd + 1;
            end

        end

        % Minimum and maximum bounds on states and controls
            % acceleration is limited by ISO acceleration constraint
        G(cstrInd,o+scurr) = 1;         % s
        g_lb(cstrInd) = s_min;
        g_ub(cstrInd) = s_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+vcurr) = 1;         % v
        g_lb(cstrInd) = v_min;
        g_ub(cstrInd) = v_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_v) = 1;          % xi_v
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_h) = 1;          % xi_h
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_s) = 1;          % xi_s
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_f) = 1;          % xi_f
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
    end

    % Move blocking
    if Mb(k) == 1
        % block the move: previous acceleration must be equal to current acceleration
        G(cstrInd,o+[aprev,acurr]) = [1, -1];
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
    end

    % ISO acceleration limits
    G(cstrInd,o+[acurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = a_max_est(k);
        cstrInd = cstrInd + 1;
    G(cstrInd,o+[acurr,xi_f]) = [1,1];
    g_lb(cstrInd) = a_min_est(k);
    g_ub(cstrInd) = inf;
        cstrInd = cstrInd + 1;

    % ISO jerk limits
    if kk > 0 || solverToUse == 2
        % HPIPM or k > 0
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = T*j_max_est(k);
            cstrInd = cstrInd + 1;
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,1];
        g_lb(cstrInd) = T*j_min_est(k);
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
    else
        % qpOASES and k == 0
        G(cstrInd,o+[acurr,xi_f]) = [1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = T*j_max_est(k)+a_minus1;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+[acurr,xi_f]) = [1,1];
        g_lb(cstrInd) = T*j_min_est(k)+a_minus1;
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
    end
    
    % Speed limit
    G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_lim_max(k);
        cstrInd = cstrInd + 1;

    % max velocity for safe/comfortable curve negotitation
    G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_curv_max(k);
        cstrInd = cstrInd + 1;

    % max velocity to simulate a stop constraint
    G(cstrInd,o+[vcurr,xi_s]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_stop_max(k);
        cstrInd = cstrInd + 1;

    % max velocity to simulate a traffic light constraint
    G(cstrInd,o+[vcurr,xi_s]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_TL_max(k);
        cstrInd = cstrInd + 1;

    % minimum velocity incentive
    G(cstrInd,o+[vcurr,xi_v]) = [1,1];
    g_lb(cstrInd) = v_minIncentive(k);
    g_ub(cstrInd) = inf;
        cstrInd = cstrInd + 1;

    % safe headway distance
    G(cstrInd,o+[scurr,xi_s])= [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k) - h_min;
        cstrInd = cstrInd + 1;
    G(cstrInd,o+[scurr,vcurr,xi_s])= [1,tau_min,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k);
        cstrInd = cstrInd + 1;

    % desired headway distance policy
    T_hwp = 2;
    A_hwp = 2;
    G_hwp = -0.0246*T_hwp + 0.010819;
    G(cstrInd,o+[scurr,vcurr,xi_h])= [1, T_hwp + G_hwp*v_est(k), -1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k) - A_hwp;
        cstrInd = cstrInd + 1;

end

% Final stage:
T = Tvec(end);
o = N*n_x_u;

% safe headway distance
G(cstrInd,o+scurr)= 1;
g_lb(cstrInd) = -inf;
g_ub(cstrInd) = s_tv_est(N) - h_min;
    cstrInd = cstrInd + 1;
G(cstrInd,o+[scurr,vcurr])= [1, tau_min];
g_lb(cstrInd) = -inf;
g_ub(cstrInd) = s_tv_est(N);
    cstrInd = cstrInd + 1;

end