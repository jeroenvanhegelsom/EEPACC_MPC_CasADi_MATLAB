function optSol = RunOpt_NLP(OPTsettings)
% RunOpt_NLP_OCP Runs the nonlinear optimization over the simulation
%
%   Inputs:
%       OPTsettings : struct with settings for the optimization
%
%   Output:
%       optSol      : struct containing the optimal solution, also
%                     including other relevant data

%% Import CasADi

import casadi.*

%% Unpack settings

% Optimization settings
W                    = OPTsettings.W_NLP;
b_quadr              = OPTsettings.b_quadr;
b_fifthOrder         = OPTsettings.b_fifthOrder;
useFifthOrderFit_NLP = OPTsettings.useFifthOrderFit_NLP;
shootingMethod       = OPTsettings.shootingMethod;
discretizationMethod = OPTsettings.discretizationMethod;
Tvec                 = OPTsettings.Tvec;
t_sim                = OPTsettings.t_sim;
s_0                  = OPTsettings.s_init;
v_0                  = OPTsettings.v_init;
s_goal               = OPTsettings.s_goal;

% Route settings
s_speedLim           = OPTsettings.s_speedLim;
v_speedLim           = OPTsettings.v_speedLim;
s_curv               = OPTsettings.s_curv;
curvature            = OPTsettings.curvature;
s_slope              = OPTsettings.s_slope;
slope                = OPTsettings.slope;
stopLoc              = OPTsettings.stopLoc;
stopRefDist          = OPTsettings.stopRefDist;
stopRefVelSlope      = OPTsettings.stopRefVelSlope;
stopVel              = OPTsettings.stopVel;
TLLoc                = OPTsettings.TLLoc;
TLstopVel            = OPTsettings.TLstopVel;
alpha_TTL            = OPTsettings.alpha_TTL;

% Preceding vehicle settings
s_tv                 = OPTsettings.s_tv;
h_min                = OPTsettings.h_min;
tau_min              = OPTsettings.tau_min;

Ts = Tvec(1);

V = SetVehicleParameters();

% Retrieve weights
w_P = W(1);
w_a = W(2);
w_j = W(3);
w_v = W(4);
w_h = W(5);
w_s = W(6);
w_f = W(7);

%% Construct use case parameters

%  === Lookup table for the road slope angle ===

    slopeLookup = casadi.interpolant('LUT','linear',{s_slope},slope);

%  === Lookup table for speed limits ===

    speedLimLookup = casadi.interpolant('LUT','linear',{s_speedLim},v_speedLim);

%  === Lookup tables for curve speed negotiation ===

    curvatureLookup = casadi.interpolant('LUT','linear',{s_curv},curvature);

%  === Lookup tables for acceleration and jerk bounds ===

    % lookup tables for speed based acceleration bounds from the ISO norm
    aMinLookup = casadi.interpolant('LUT','linear',{[0,5,20,25]},[-4,-4,-2,-2]);
    aMaxLookup = casadi.interpolant('LUT','linear',{[0,5,20,25]},[5,5,3.5,3.5]);

    % lookup table for speed based jerk bounds from the ISO norm
    jMinMaxMagLookup = casadi.interpolant('LUT','linear',{[0,5,20,25]},[5,5,2.5,2.5]);

% === Lookup table for stops ===

    % initialize
    sStopVel = [];
    vStopVel = [];
    stopLocs = sort(stopLoc);
    stopRefvelIncr = stopRefDist*stopRefVelSlope;

    % set velocity profile
    for i = 1:length(stopLocs)
        sStopVel = [sStopVel, stopLocs(i)-stopRefDist, stopLocs(i), stopLocs(i)+stopRefDist];
        vStopVel = [vStopVel,          stopRefvelIncr,     stopVel,          stopRefvelIncr];
    end

    % velocity profile correction in case consecutive stops are close to each other
    for i = 1:length(vStopVel)-1
        if sStopVel(i+1) <= sStopVel(i)
            stopDistCorr  = .5*(sStopVel(i) - sStopVel(i+1)) + sStopVel(i+1);
            vStopVel(i)   = stopRefvelIncr/(1 + stopRefDist/( sStopVel(i) - sStopVel(i+1) ));
            vStopVel(i+1) = stopRefvelIncr/(1 + stopRefDist/( sStopVel(i) - sStopVel(i+1) ));
            sStopVel(i)   = stopDistCorr - 1;
            sStopVel(i+1) = stopDistCorr + 1;
        end
    end

    % create lookup table
    if length(stopLocs)<1
        sStopVel = [0,1];
        vStopVel = [1e5,1e5];
    end
        
    stopMaxVelLookup = casadi.interpolant('LUT','linear',{sStopVel},vStopVel);

% === Lookup tables for traffic lights === 

    % check if there are traffic lights
    if isempty(TLLoc)
        % no traffic lights
        tlMaxVelLookup{1} = casadi.interpolant('LUT','linear',{[0,1]},[1e3,1e3]);
        tlSPATLookup{1}   = casadi.interpolant('LUT','linear',{[0,1]},[1e3,1e3]);
    else
        % there are traffic lights

        % extract data
        TLlocs  = TLLoc(:,1);
        TLSPAT  = TLLoc(:,2:4);
    
        % looping over the traffic lights
        sTLVel  = zeros(length(TLlocs),3);
        vTLVel = [stopRefvelIncr, TLstopVel, stopRefvelIncr];
        for i = 1:length(TLlocs)
            
            tSPAT   = [];
            TLState = [];
    
            % set velocity profile
            sTLVel(i,:) = [TLlocs(i)-stopRefDist, TLlocs(i), TLlocs(i)+stopRefDist];
            tlMaxVelLookup{i} = casadi.interpolant('LUT','linear',{sTLVel(i,:)},vTLVel);
            % loop through horizon to set the timing
            for j = 1:(t_sim/Ts+1)
                tSPAT   = [tSPAT, (j-1)*Ts];
                if mod((j-1)*Ts-TLSPAT(i,1),sum(TLSPAT(i,2:3))) < TLSPAT(i,2)
                    TLState = [TLState, .2];     % red
                else
                    TLState = [TLState, 1e3];  % green
                end
            end
    
            % create lookup table
            tlSPATLookup{i} = casadi.interpolant('LUT','linear',{tSPAT} ,TLState);
    
        end
    end

% === Minimum velocity incentive ===
    
    % get the minimum of all maximum speeds excluding stops
    [s_velInc,v_velInc] = minPWA(s_speedLim, v_speedLim, s_curv, alpha_TTL.*(abs(curvature)).^(-1/3));

    % saturate slopes
    [s_velInc,v_velInc] = SaturateSlopePWA(s_velInc,v_velInc,0.5);

    % delete duplicates
    s_minVel_ = s_velInc;
    v_minVel_ = v_velInc;
    pointsToKeep = ~diff(s_velInc)==0;
    s_velInc = [s_velInc(pointsToKeep),s_minVel_(end)];
    v_velInc = [v_velInc(pointsToKeep),v_minVel_(end)];

    % simplify
    [s_velInc,v_velInc] = SimplifyPWA(s_velInc,v_velInc);

    % rescale
    v_velInc = 1.0*(v_velInc);

    % save for plotting
    optSol.s_velInc = s_velInc;
    optSol.v_velInc = v_velInc;

    % create lookup table
    velIncentiveLookup = casadi.interpolant('LUT','linear',{s_velInc},v_velInc);

%% Set optimization settings 

% simulation variables;
N = t_sim/Ts; 	                        % total number of discrete steps               (-)

% bounds on the state and control variables
s_min    =  0;                          % minimum distance (= pos relative to origin)  (m)
s_max    =  s_goal;                     % maximum distance                             (m)
v_min    =  0;                          % minimum velocity                             (m/s)
v_max    =  V.v_max;                    % maximum velocity                             (m/s)
Fm_min   = -V.phi*V.T_m_max/V.eta_TF;   % minimum traction force                       (N)
Fm_max   =  V.phi*V.T_m_max*V.eta_TF;   % maximum traction force                       (N)
Fb_min   = -inf;                        % minimum mechanical braking force             (N)
Fb_max   =  0;                          % maximum mechanical braking force             (N)

% state variable declaration
s      = SX.sym('s');
v      = SX.sym('v');
theta  = SX.sym('theta');
j      = SX.sym('j');
x      = [  s;   v;            theta; j];
x_init = [s_0; v_0; slopeLookup(s_0); 0];

% control variable declaration
Fm = SX.sym('Fm');
Fb = SX.sym('Fb');

% slack variables
xi_v = SX.sym('xi_v');
xi_h = SX.sym('xi_h');
xi_s = SX.sym('xi_s');
xi_f = SX.sym('xi_f');

u = [Fm; Fb; xi_v; xi_h; xi_s; xi_f];
n_slackVars = length(u) - 2;

% model dynamics
xdot = [v; 1/(V.lambda*V.m)*(Fm + Fb - V.zeta_a*v^2-V.c_r*V.m*V.g*cos(theta) - V.m*V.g*sin(theta)); 0; 0];

% Model definitions
rpm = (30/pi)*V.phi*v;

if useFifthOrderFit_NLP
    b = b_fifthOrder;
    P_bat = b(1) + b(2)*Fm + b(3)*rpm + b(4)*Fm^2 + b(5)*Fm*rpm + b(6)*rpm^2 + ...
            b(7)*Fm^3 + b(8)*Fm^2*rpm + b(9)*Fm*rpm^2 + b(10)*rpm^3 + b(11)*Fm^4 + ...
            b(12)*Fm^3*rpm + b(13)*Fm^2*rpm^2 + b(14)*Fm*rpm^3 + b(15)*rpm^4 + ...
            b(16)*Fm^5 + b(17)*Fm^4*rpm + b(18)*Fm^3*rpm^2 + b(19)*Fm^2*rpm^3 + ...
            b(20)*Fm*rpm^4 + b(21)*rpm^5;
else
    b = b_quadr;
    P_bat = b(1) + b(2)*Fm + b(3)*rpm + b(4)*Fm.^2 + b(5)*Fm*rpm + b(6)*rpm^2;
end

a = xdot(2);

% Objective terms
L = w_P*P_bat + w_a*(a)^2 + w_j*(j)^2 + w_v*xi_v + w_h*(xi_h^2 + 1e2*xi_h) + w_s*xi_s + w_f*xi_f;

opts = struct;
opts.ipopt.max_iter                     = OPTsettings.NLPmaxIter;
opts.ipopt.print_level                  = 3;
opts.print_time                         = 1;
opts.ipopt.acceptable_tol               = 1e-8;
opts.ipopt.acceptable_obj_change_tol    = 1e-6;

%% Problem formulation initialization

n_x = length(x);
n_u = length(u);
n_x_u = n_u + n_x;

%% Formulate discrete dynamics

% Continuous time dynamics
f     = Function('f', {x, u}, {xdot, L});
X0    = MX.sym('X0', n_x);
U     = MX.sym('U',  n_u);
X     = X0;
Q     = 0;

if discretizationMethod == 0
    % First order Euler method
    [DX,DQ] = f(X, U);
    X = X + Ts*DX;
    Q = Q + Ts*DQ;
else
    % Fixed step Runge-Kutta 4 integrator
    M  = 4; % RK4 steps per interval
    DT = Ts/M;
    for ii=1:M
       [k1, k1_q] = f(X, U);
       [k2, k2_q] = f(X + DT/2 * k1, U);
       [k3, k3_q] = f(X + DT/2 * k2, U);
       [k4, k4_q] = f(X + DT   * k3, U);
       X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4);
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
end   

F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

%% Construct the problem

% Start with an empty NLP
z   = [];
z0  = [];
lbz = [];
ubz = [];
J   = 0;
g   = [];
lbg = [];
ubg = [];

if shootingMethod == 0

    X0 = x_init;

elseif shootingMethod == 1

    % "Lift" initial conditions
    X0 = MX.sym('X0', n_x);
    z = [z; X0];
    lbz = [lbz; x_init];
    ubz = [ubz; x_init];
    z0  = [z0;  x_init];

else
    
    error('unknown shooting method');
    
end

% State and control indices to simplify formulation of the constraints
i_s   = 1;      % travel distance                    s 
i_v   = 2;      % velocity                           v
i_th  = 3;      % road slope angle                   theta
i_j   = 4;      % jerk                               j
i_Fm  = 1;      % motor force                        F_m
i_Fb  = 2;      % mechanical braking force           F_b
i_xiv = 3;      % velocity incentive slack variable  xi_v
i_xih = 4;      % headway policy slack variable      xi_h
i_xis = 5;      % safet-critical feasibility slack   xi_s
i_xif = 6;      % feasibility slack                  xi_f

% Formulate the NLP
Xk = X0;
Uk = zeros(n_u,1);
for k = 0:N-1
    Uk_prev = Uk;
    Xk_prev = Xk;

    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)], n_u);
    z   = [z; Uk];
    
    % optimization variable constraints
    lbz = [lbz; Fm_min; Fb_min;    zeros(n_slackVars,1)];
    ubz = [ubz; Fm_max; Fb_max; inf*ones(n_slackVars,1)];
    z0  = [z0;  zeros(n_u,1)];
    
    % Integrate till the end of the interval
    Fk      = F('x0', Xk, 'p', Uk);
    J       = J + Fk.qf;
    
    if shootingMethod == 0
        % single shooting

        Xk = Fk.xf;
        
        % Add inequality constraint for the states
        lbg = [lbg;   s_min;   v_min;     -inf;    -inf];
        g   = [g;   Xk(i_s); Xk(i_v); Xk(i_th); Xk(i_j)];
        ubg = [ubg;   s_max;   v_max;      inf;     inf];
        
    else  
        % multiple shooting

        Xk_end = Fk.xf;
        
        % New NLP variable for state at end of interval
        Xk  = MX.sym(['X_' num2str(k+1)], n_x);
        z   = [z; Xk];
        
        % Add bounds and initial conditions for the states
        %             s      v   theta    j
        lbz = [lbz; s_min; v_min; -inf; -inf];
        ubz = [ubz; s_max; v_max;  inf;  inf];
        z0  = [z0;      0;     0;    0;    0];
        
        % Add equality constraints for continuity of s and v
        lbg = [lbg; 0; 0];
        g   = [g; Xk_end(i_s) - Xk(i_s); Xk_end(i_v) - Xk(i_v)];
        ubg = [ubg; 0; 0];  

    end

    % Set the road angle and jerk states
    lbg = [lbg; 0];
    if sum(slope) < 1e-1
        g = [g; Xk(i_th)];
    else
        g = [g; Xk(i_th) - slopeLookup(Xk(i_s))];
    end
    ubg = [ubg; 0];
    % ===
    lbg = [lbg; 0];
    g   = [g; Xk(i_j) - (        Uk(i_Fm) +      Uk(i_Fb) - V.zeta_a*     Xk(i_v)^2 - V.c_r*V.m*V.g*cos(     Xk(i_th)) - V.m*V.g*sin(     Xk(i_th)) ...
                          - Uk_prev(i_Fm) - Uk_prev(i_Fb) + V.zeta_a*Xk_prev(i_v)^2 + V.c_r*V.m*V.g*cos(Xk_prev(i_th)) + V.m*V.g*sin(Xk_prev(i_th)))/(V.lambda*V.m*Ts)];       
    ubg = [ubg; 0];
    
    % Add motor force bounds for omega>omega^*
    lbg = [lbg; 0];
    g   = [g; Uk(i_Fm)*Xk(i_v) + V.P_m_max/V.eta_TF + Uk(i_xif)];      
    ubg = [ubg; inf];
    % ===
    lbg = [lbg; -inf];
    g   = [g; Uk(i_Fm)*Xk(i_v) - V.P_m_max*V.eta_TF - Uk(i_xif)]; 
    ubg = [ubg; 0];

    % Total traction force constraint (based on maximum total friction)
    lbg = [lbg;    0];
    g   = [g; Uk(i_Fm) + Uk(i_Fb) + V.mu*V.m*V.g*cos(Xk(i_th)) + Uk(i_xif)];
    ubg = [ubg;  inf];
    % ===
    lbg = [lbg; -inf];
    g   = [g; Uk(i_Fm) + Uk(i_Fb) - V.mu*V.m*V.g*cos(Xk(i_th)) - Uk(i_xif)];
    ubg = [ubg;    0];

    % Total motor force constraint (based on maximum rear tire friction)
    lbg = [lbg;    0];
    g   = [g; V.L/(V.mu*V.m)*Uk(i_Fm) + V.h_g*V.lambda*...
                (Uk(i_Fm) + Uk(i_Fb) - V.zeta_a*Xk(i_v)^2 - V.c_r*V.m*V.g*cos(Xk(i_th)) - V.m*V.g*sin(Xk(i_th)))/(V.lambda*V.m) ... 
                + V.h_g*V.zeta_a/V.m*Xk(i_v)^2 + V.g*(V.L_f*cos(Xk(i_th)) + V.h_g*sin(Xk(i_th))) + Uk(i_xif)];
    ubg = [ubg;  inf];
    % ===
    lbg = [lbg; -inf];
    g   = [g; V.L/(V.mu*V.m)*Uk(i_Fm) - V.h_g*V.lambda*...
                (Uk(i_Fm) + Uk(i_Fb) - V.zeta_a*Xk(i_v)^2 - V.c_r*V.m*V.g*cos(Xk(i_th)) - V.m*V.g*sin(Xk(i_th)))/(V.lambda*V.m) ... 
                - V.h_g*V.zeta_a/V.m*Xk(i_v)^2 - V.g*(V.L_f*cos(Xk(i_th)) + V.h_g*sin(Xk(i_th))) - Uk(i_xif)];
    ubg = [ubg;    0];

    % ISO acceleration constraints
    lbg = [lbg; 0];
    g   = [g; ( Uk(i_Fm) + Uk(i_Fb) - V.zeta_a*Xk(i_v)^2 - V.c_r*V.m*V.g*cos(Xk(i_th)) - V.m*V.g*sin(Xk(i_th)) )/(V.lambda*V.m) ...
                - aMinLookup(Xk(i_v)) + Uk(i_xif)];
    ubg = [ubg; inf];
    % ===
    lbg = [lbg; -inf];
    g   = [g; ( Uk(i_Fm) + Uk(i_Fb) - V.zeta_a*Xk(i_v)^2 - V.c_r*V.m*V.g*cos(Xk(i_th)) - V.m*V.g*sin(Xk(i_th)) )/(V.lambda*V.m) ...
                - aMaxLookup(Xk(i_v)) - Uk(i_xif)];
    ubg = [ubg; 0];

    % ISO jerk constraints
    lbg = [lbg; 0;];
    g   = [g; Xk(i_j) + jMinMaxMagLookup(Xk(i_v)) + Uk(i_xif)];
    ubg = [ubg; inf];
    % ===
    lbg = [lbg; -inf];
    g   = [g; Xk(i_j) - jMinMaxMagLookup(Xk(i_v)) - Uk(i_xif)];
    ubg = [ubg; 0];

    % Road speed limit
    lbg = [lbg; -inf];
    g   = [g; Xk(i_v) - speedLimLookup(Xk(i_s)) - Uk(i_xif)];
    ubg = [ubg; 0];

    % Curve velocity based on the Two-Thirds law
    lbg = [lbg; -inf];
    g   = [g; Xk(i_v) - alpha_TTL*(abs(curvatureLookup(Xk(i_s))))^(-1/3) - Uk(i_xif)];
    ubg = [ubg;    0];

    % Stopping constraints
    lbg = [lbg; -inf];
    g   = [g; Xk(i_v) - stopMaxVelLookup(Xk(i_s)) - Uk(i_xis)];
    ubg = [ubg;    0];

    % Traffic light constraints
    if ~isempty(TLLoc)
        for i = 1:length(TLlocs)
            LUT_TL_timing = tlSPATLookup{i};
            LUT_TL_vProfile = tlMaxVelLookup{i};
    
            % red phase: stopping constraint
            lbg = [lbg; -inf];
            g   = [g; Xk(i_v) - LUT_TL_vProfile(Xk(i_s)) - LUT_TL_timing(k*Ts) - Uk(i_xis)];
            ubg = [ubg;    0];

            % green phase: driving incentive
            lbg = [lbg;   0];
            g   = [g; Xk(i_v) + LUT_TL_vProfile(Xk(i_s)) + 1e3 - 10 - LUT_TL_timing(k*Ts) + Uk(i_xis)];
            ubg = [ubg; inf];
        end
    end

    % Minimum velocity incentive
    lbg = [lbg; 0];
    g   = [g; Xk(i_v) - velIncentiveLookup(Xk(i_s)) + Uk(i_xiv)];
    ubg = [ubg; inf];
     
    % Vehicle following constraint: minimum headway
    lbg = [lbg;              -inf;                                  -inf];
    g   = [  g;           Xk(i_s); Xk(i_s) + tau_min*Xk(i_v) - Uk(i_xis)];
    ubg = [ubg; s_tv(k+1) - h_min;                             s_tv(k+1)];

    % Vehicle following constraint: desired headway
    A_hwp = 2;
    T_hwp = 2;
    G_hwp = -0.0246*T_hwp + 0.010819;
    lbg = [lbg; -inf];
    g   = [g; Xk(i_s) + Xk(i_v)*T_hwp + Xk(i_v)^2*G_hwp - Uk(i_xih)];
    ubg = [ubg; s_tv(k+1) - A_hwp];

end

%% Create an NLP solver and solve the problem

prob = struct('f', J, 'x', z, 'g', g);
solver = nlpsol('solver', 'ipopt', prob, opts);

% Solve the NLP
tSolveTic = tic();
sol = solver('x0', z0, 'lbx', lbz, 'ubx', ubz, 'lbg', lbg, 'ubg', ubg);
optSol.tSolve = toc(tSolveTic);
optSol.exitMessage = solver.stats().return_status;
z_opt = full(sol.x);

%% Retrieve the solution variables
if shootingMethod == 0
    u_opt = reshape(z_opt,[n_u,length(z_opt)/n_u]);
    x_opt = x_init;
    for k = 0:N-1
        Fk = F('x0', x_opt(:,end), 'p', u_opt(:,k+1));
        x_opt = [x_opt, full(Fk.xf)];
    end
    x_opt = full(x_opt);
    u_opt = full(u_opt);
    s_opt     = x_opt(1,:)';
    v_opt     = x_opt(2,:)';
    theta_opt = x_opt(3,:)';
    j_opt     = x_opt(4,:)';
    Fm_opt    = u_opt(1,:)';
    Fb_opt    = u_opt(2,:)';
    xi_v_opt  = u_opt(3,:)';
    xi_h_opt  = u_opt(4,:)';
    xi_s_opt  = u_opt(5,:)';
    xi_f_opt  = u_opt(6,:)';
else    
    s_opt     = full(z_opt(1:n_x_u:end));
    v_opt     = full(z_opt(2:n_x_u:end));
    theta_opt = full(z_opt(3:n_x_u:end));
    j_opt     = full(z_opt(4:n_x_u:end));
    Fm_opt    = full(z_opt(5:n_x_u:end));  
    Fb_opt    = full(z_opt(6:n_x_u:end));  
    xi_v_opt  = full(z_opt(7:n_x_u:end));  
    xi_h_opt  = full(z_opt(8:n_x_u:end));  
    xi_s_opt  = full(z_opt(9:n_x_u:end));
    xi_f_opt  = full(z_opt(10:n_x_u:end)); 
end

%% Calculate other optimal quantities
rpm_opt = (30/pi)*v_opt(1:end-1)*V.phi;
P_opt = GetMotorPower_FifthOrderSurface(Fm_opt,rpm_opt,b_fifthOrder);
E_opt = zeros(length(P_opt),1);
for k = 1:length(P_opt)
    E_opt(k) = Ts*sum(P_opt(1:k));
end
a_opt = diff(v_opt)/Ts;
Tm_opt = Fm_opt./V.phi./(V.eta_TF.^sign(Fm_opt));

%% prepare output struct

% optimized variables
optSol.s_opt        = s_opt;
optSol.v_opt        = v_opt;
optSol.theta_opt    = theta_opt;
optSol.j_opt        = j_opt;
optSol.Fm_opt       = Fm_opt;
optSol.Fb_opt       = Fb_opt;
optSol.xi_v_opt     = xi_v_opt;
optSol.xi_h_opt     = xi_h_opt;
optSol.xi_s_opt     = xi_s_opt;
optSol.xi_f_opt     = xi_f_opt;

% derived from optimal variables
optSol.P_opt     = P_opt;
optSol.E_opt     = E_opt;
optSol.a_opt     = a_opt;
optSol.Tm_opt    = Tm_opt;
optSol.rpm_opt   = rpm_opt;

% extract weights
w_P     = W(1);
w_a     = W(2);
w_j     = W(3);
w_xi_v  = W(4);
w_xi_h  = W(5);
w_xi_s  = W(6);
w_xi_f  = W(7);

% costs over the horizon
for k = 1:N
    cost_P(k)    = w_P*sum(P_opt(1:k));
    cost_a(k)    = w_a*sum(a_opt(1:k).^2);
    cost_j(k)    = w_j*sum(j_opt(1:k).^2);
    cost_xi_v(k) = w_xi_v*sum(xi_v_opt(1:k));
    cost_xi_h(k) = w_xi_h*sum(xi_h_opt(1:k));
    cost_xi_s(k) = w_xi_s*sum(xi_s_opt(1:k));
    cost_xi_f(k) = w_xi_f*sum(xi_f_opt(1:k));
end
optSol.cost_P    = cost_P;
optSol.cost_a    = cost_a;
optSol.cost_j    = cost_j;
optSol.cost_xi_v = cost_xi_v;
optSol.cost_xi_h = cost_xi_h;
optSol.cost_xi_s = cost_xi_s;
optSol.cost_xi_f = cost_xi_f;

end