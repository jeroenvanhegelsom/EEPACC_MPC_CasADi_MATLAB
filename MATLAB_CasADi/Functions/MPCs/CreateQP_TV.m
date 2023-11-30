function [H,c,G,z_lb,z_ub,g_lb,g_ub] = CreateQP_TV(OPTsettings,n_x,n_u,s_0,v_0,s_est,t_0,a_minus1)
% CreateQP_TV Creates the target vehicle QP for qpOASES and HPIPM, i.e. it builds the matrices and vectors that define the optimization problem
%
%   Inputs:
%       OPTsettings : struct with settings for the optimization
%       n_x         : number of states in each time step
%       n_u         : number of controls in each time step
%       s_0         : initial travel distance for the QP
%       v_0         : initial velocity for the QP
%       s_est       : estimated travel distance over the horizon
%       v_est       : estimated velocity over the horizon
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

solverToUse         = OPTsettings.solverToUse;
W                   = OPTsettings.W_TV;
s_goal              = OPTsettings.s_goal;
Ts                  = OPTsettings.TV_Ts;
N                   = OPTsettings.TV_N_hor;

% get vehicle parameter struct
V = SetVehicleParameters();

% extract weights
w_v = W(1);
w_a = W(2);
w_j = W(3);
w_f = W(4);

%% Initialize

% Estimate bounds on velocity from the route
v_est = zeros(N,1);
[~,v_lim_max,v_stop_max,v_TL_max,v_curv_max,a_min_est,a_max_est,j_min_est,j_max_est] = EstimateRouteAndComfortBounds(OPTsettings, 2, s_est, v_est, t_0, N);

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
    z_lb(1:n_x_u:(n_x_u*N+n_x)) = s_min;
    z_ub(1:n_x_u:(n_x_u*N+n_x)) = s_max;
    z_lb(2:n_x_u:(n_x_u*N+n_x)) = v_min;
    z_ub(2:n_x_u:(n_x_u*N+n_x)) = v_max;
    z_lb(3:n_x_u:(n_x_u*N+n_x)) = a_min;
    z_ub(3:n_x_u:(n_x_u*N+n_x)) = a_max;
    z_lb(4:n_x_u:(n_x_u*N+n_x)) = a_min;
    z_ub(4:n_x_u:(n_x_u*N+n_x)) = a_max;
    for i = 1:n_u-1 % loop over slack variables
        z_lb((4+i):n_x_u:(n_x_u*N+n_x)) = slack_min;
        z_ub((4+i):n_x_u:(n_x_u*N+n_x)) = slack_max;
    end
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
    xi_f    = 5;        % feasibility slack variable         xi_f
else
% qpOASES
    % indices of z (mod n_x_u + 1):   
    scurr   = 1;        % distance                           s 
    vcurr   = 2;        % velocity                           v
    aprev   = 3-n_x_u;  % previous acceleration              a_prev
    acurr   = 3;        % acceleration                       a_curr
    xi_f    = 4;        % feasibility slack variable         xi_f
end

cstrInd = 1;
for kk = 0:N-1

    % k is the index for the estimates (1 to N), kk is the true time step index (0 to N-1)
    k = kk+1;

    % offset to simplify notation
    o = kk*n_x_u;

    %% === Objective function ===

    % objective function: velocity term
    c(o+vcurr) = c(o+vcurr) - w_v;

    % objective function: acceleration term
    ii = o+acurr;
    H(ii,ii) = H(ii,ii) + 2*w_a;
    
    % objective function: jerk term
    if kk == 0
        ii = o+acurr;
        H(ii,ii) = H(ii,ii) + 2*w_j/Ts^2;
        c(ii) = c(ii) -2*w_j/Ts*a_minus1;
    else
        ii = o + [acurr,aprev];
        H(ii,ii) = H(ii,ii) + 2*w_j/Ts^2*[1, -1; -1, 1];
    end

    % objective function: slack variable
    c(o+xi_f) = c(o+xi_f) + w_f;
    
   %% === Constraints ===

    if solverToUse == 2
    % HPIPM

        % continuity of the state variables
        ii = o+[scurr,vcurr,acurr,n_x_u+[scurr,vcurr,aprev]];
        G(cstrInd,ii) = [1, Ts, .5*Ts^2, -1,  0,  0]; % continuity of distance
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
        G(cstrInd,ii) = [0,  1,  Ts,  0, -1,  0]; % continuity of velocity
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
            G(cstrInd,ii) = [1, Ts, .5*Ts^2, -1,  0]; % continuity of distance
            g_lb(cstrInd) = 0;
            g_ub(cstrInd) = 0;
                cstrInd = cstrInd + 1;
            G(cstrInd,ii) = [0,  1, Ts,  0, -1]; % continuity of velocity
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
        G(cstrInd,o+xi_f) = 1;          % xi_f
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
    end

    % acceleration bounds
    G(cstrInd,o+[acurr,xi_f]) = [1,1];
    g_lb(cstrInd) = a_min_est(k);
    g_ub(cstrInd) = inf;
        cstrInd = cstrInd + 1;
    G(cstrInd,o+[acurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = a_max_est(k);
        cstrInd = cstrInd + 1;

    % jerk bounds
    if kk > 0 || solverToUse == 2
        % HPIPM or k > 0
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,1];
        g_lb(cstrInd) = Ts*j_min_est(k);
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = Ts*j_max_est(k);
            cstrInd = cstrInd + 1;
    else
        % qpOASES and k == 0
        G(cstrInd,o+[acurr,xi_f]) = [1,1];
        g_lb(cstrInd) = Ts*j_min_est(k)+a_minus1;
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
            G(cstrInd,o+[acurr,xi_f]) = [1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = Ts*j_max_est(k)+a_minus1;
            cstrInd = cstrInd + 1;
    end
    
    % Speed limit
    G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  .8*v_lim_max(k);
        cstrInd = cstrInd + 1;

    % max velocity for safe/comfortable curve negotitation
    G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  .8*v_curv_max(k);
        cstrInd = cstrInd + 1;

    % max velocity to simulate a stop constraint
    G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_stop_max(k);
        cstrInd = cstrInd + 1;

    % max velocity to simulate a traffic light constraint
    G(cstrInd,o+[vcurr,xi_f]) =  [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) =  v_TL_max(k);
        cstrInd = cstrInd + 1;

end

% final offset
o = N*n_x_u;

% objective function: distance term
c(o+vcurr) = c(o+vcurr) - w_v;

end