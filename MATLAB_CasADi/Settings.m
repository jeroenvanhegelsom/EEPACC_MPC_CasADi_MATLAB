function OPTsettings = Settings(OPTsettings)
% Settings Generates the OPTsettings struct with settings for the NLP, MPCs and their use case
%
%   Inputs:
%       OPTsettings : struct with initial settings
%
%   Outputs:
%       OPTsettings : struct with generated settings

%% Objective weights for tuning

% Objective function weights: NLP
w_P = 220;
w_c = 0.3;
w_v = 4e5;
w_h = 8e3;
w_f = 1e7;
wc_to_wa = 3*1e5;
wc_to_wj = 1*1e7;
wf_to_ws = 9e0;
W(1) = w_P;                 % w_P: weight of the power consumption
W(2) = w_c*wc_to_wa;        % w_a: weight of the acceleration penalty
W(3) = w_c*wc_to_wj;        % w_j: weight of the jerk penalty
W(4) = w_v;                 % w_v: weight of the slack variable for minimum velocity travel incentive
W(5) = w_h;                 % w_h: weight of slack variable for vehicle following
W(6) = w_f*wf_to_ws;        % w_s: weight of slack variable for safety-critical feasibility
W(7) = w_f;                 % w_f: weight of slack variable for feasibility
OPTsettings.W_NLP = W;

% Objective function weights: Force-based MPC
w_P = 50;
w_c = 0.3;
w_v = 4e5;
w_h = 8e3;
w_f = 1e7;
wc_to_wa = 3*1e5;
wc_to_wj = 1*1e7;
wf_to_ws = 9e0;
W(1) = w_P;                 % w_P: weight of the power consumption
W(2) = w_c*wc_to_wa;        % w_a: weight of the acceleration penalty
W(3) = w_c*wc_to_wj;        % w_j: weight of the jerk penalty
W(4) = w_v;                 % w_v: weight of the slack variable for minimum velocity travel incentive
W(5) = w_h;                 % w_h: weight of slack variable for vehicle following
W(6) = w_f*wf_to_ws;        % w_s: weight of slack variable for safety-critical feasibility
W(7) = w_f;                 % w_f: weight of slack variable for feasibility
OPTsettings.W_FB = W;

% Objective function weights: Acceleration-based MPC
w_c = 0.1;
w_v = 8e5;
w_h = 1e4;
w_f = 1e10;
wc_to_wa = 3*1e5;
wc_to_wj = 1*1e7;
wf_to_ws = 9e0;
W(1) = w_c*wc_to_wa;        % w_a: weight of the acceleration penalty
W(2) = w_c*wc_to_wj;        % w_j: weight of the jerk penalty
W(3) = w_v;                 % w_v: weight of the slack variable for minimum velocity travel incentive
W(4) = w_h;                 % w_h: weight of slack variable for vehicle following
W(5) = w_f*wf_to_ws;        % w_s: weight of slack variable for safety-critical feasibility
W(6) = w_f;                 % w_f: weight of slack variable for feasibility
OPTsettings.W_AB = 1e-3*W;

% Objective function weights: Baseline MPC
W(1) = 1e2;                 % w_s : weight of the travel incentive
W(2) = 0;                   % w_a : weight of the acceleration penalty
W(3) = 0;                   % w_j : weight of the jerk penalty
W(4) = 1e7;                 % w_f : weight of the slack variable for feasibility
OPTsettings.W_BL = W;

% Objective function weights: Target vehicle MPC
W(1) = 1e2;                 % w_s : weight of the travel incentive
W(2) = 0;                   % w_a : weight of the acceleration penalty
W(3) = 0;                   % w_j : weight of the jerk penalty
W(4) = 1e7;                 % w_f : weight of the slack variable for feasibility
OPTsettings.W_TV = W;

%% General settings

Ts = OPTsettings.Ts;

% initial conditions
OPTsettings.s_init           = 0;       % initial distance                                                 (m)
OPTsettings.v_init           = 0/3.6;   % initial velocity                                                 (m/s)
OPTsettings.a_minus1         = 0;       % acceleration in the step just before the MPC/NLP starts          (m/s^2)

%% NLP settings

OPTsettings.shootingMethod       = 1;       % Shooting method: single shooting (0) or multiple shooting (1)
OPTsettings.discretizationMethod = 1;       % Discretization method: first order Euler (0) or Fixed step Runge-Kutta 4 (1)
OPTsettings.NLPmaxIter           = 5e3;     % Maximum number of IPOPT iterations for the NLP optimization
OPTsettings.useFifthOrderFit_NLP = true;    % If true, the fifth order power consumption fit is used in the objective function of the NLP, if false, the quadratic fit is used

%% MPC settings

OPTsettings.FBuseTaylor = true;             % FBMPC: whether to use Taylor linearization for the dynamics and headway policy (true) or to linearize 'naively' using estimates (false)

% move-blocking
moveBlockingSettings = ones(1,20);  % [ones(1,10), 2*ones(1,10), 4*ones(1,5)];
OPTsettings.N_hor    = sum(moveBlockingSettings);

% estimation
OPTsettings.paramEstSetting      = 1;       % ego vehicle distance and velocity estimation method: constant velocity (0), constant acceleration (1), use the previous MPC solutions (2)
OPTsettings.TVestSetting         = 1;       % target vehicle distance and velocity estimation method: constant velocity (0) or constant acceleration (1)
OPTsettings.tConstACC_ego        = 3;       % if paramEstSetting=1, how long to use constant acceleration before changing to constant velocity (seconds)
OPTsettings.tConstACC_tar        = 5;       % if TVestSetting=1, how long to use constant acceleration before changing to constant velocity (seconds)

% plant simulation
OPTsettings.N_integratePlant     = 10;       % number of RK4 steps for the integration step in the MPC plant simulation (u_k -> measured x_k)

% optimization
OPTsettings.solverToUse         = 1;       % sparse qpOASES (0), dense qpOASES (1), sparse HPIPM (2).
OPTsettings.ABMPCmaxIterHPIPM   = 5e2;     % Maximum number of HPIPM iterations for the ABMPC
OPTsettings.FBMPCmaxIterHPIPM   = 5e2;     % Maximum number of HPIPM iterations for the FBMPC
OPTsettings.TVMPCmaxIterHPIPM   = 1e3;     % Maximum number of HPIPM iterations for the TVMPC

% variable time step length
    % TsMax = 1.5;
    % OPTsettings.Tvec = Ts*(TsMax/Ts).^((0:OPTsettings.N_hor-1)/(OPTsettings.N_hor-1));
OPTsettings.Tvec = Ts*ones(1,OPTsettings.N_hor);   % vector of length N_hor that has the time step lengths for each step. Tvec(1) must equal Ts

%% Baseline MPC settings (also TVMPC aspects)

OPTsettings.BLaccMin  = -2;     % minimum acceleration of the baseline ACC    (m/s^2)
OPTsettings.BLaccMax  =  2;     % maximum acceleration of the baseline ACC    (m/s^2)
OPTsettings.BLjerkMin = -1;     % minimum jerk of the baseline ACC            (m/s^3)
OPTsettings.BLjerkMax =  1;     % maximum jerk of the baseline ACC            (m/s^3)

OPTsettings.BL_a_LimLowVel  = 3.0; % acceleration limit of the BLMPC at v < 5 m/s    (m/s^2)
OPTsettings.BL_a_LimHighVel = 2.0; % acceleration limit of the BLMPC at v > 10 m/s   (m/s^2)
OPTsettings.BL_j_LimLowVel  = 3.0; % jerk         limit of the BLMPC at v < 5 m/s    (m/s^3)
OPTsettings.BL_j_LimHighVel = 1.5; % jerk         limit of the BLMPC at v > 10 m/s   (m/s^3)


OPTsettings.BL_N_hor         = OPTsettings.N_hor;%OPTsettings.N_hor;                      % horizon length in the BLMPC
OPTsettings.BL_Ts            = Ts;                                     % time step used in the BLMPC (s)
OPTsettings.BL_trajEstSett   = 1;                                      % vehicle distance and velocity estimation method for BLMPC: constant velocity (0), constant acceleration (1), use the previous MPC solutions (2)

%% Use case: route definition

OPTsettings.s_goal = inf; % maximum distance goal (m)

% resolution over distance in (m) and time in (s)
OPTsettings.sRes = 1; 
OPTsettings.tRes = 1;

% Custom use case
if OPTsettings.useCaseNum == 0

    % const. road slope           [slope (deg), start dist (m), end dist (m)]
    OPTsettings.slopes          = [];

    % === section 1: urban ===

    % speed limits:               [max speed (km/h), initial distance (m)]
    OPTsettings.speedLimZones   = [ 30,    0; 
                                    50,  600;
                                    80, 1000; 
                                   120, 4450;
                                   80, 10700]; 
    

    % curves:                     [curvature (1/m, pos -> ccw), start dist (m), end dist (m)]             
    OPTsettings.curves          = [  -1/2, 200, 206;
                                     -1/9, 2600, 2605;
                                     1/17, 2605, 2650; % roundabout 1
                                    -1/12, 2650, 2655;
                                     -1/9, 3400, 3405; % roundabout 2
                                     1/17, 3405, 3450;
                                    -1/12, 3450, 3455;
                                    -1/30, 4300, 4310; 
                                    -1/52, 4400, 4460;
                                    -1/75, 7900, 8253];
    
    % stops:                      [distance on route from start (m)]
    OPTsettings.stopLoc         = [80;
                                   450;
                                   600;
                                   4000; % traffic light
                                   10700]; % traffic light
    
    

    % traffic lights:             [distance on route from start (m), phase (s), red light time (s), green light time (s)]
    OPTsettings.TLLoc           = [];

    OPTsettings.cutOffDist = 11.5e3;

else
    % get a predefined use case
    OPTsettings = GetUseCase(OPTsettings);
end

%% Use case: other definitions

% vehicle following
OPTsettings.h_min            = 2;            % minimum headway distance                     (m)
OPTsettings.tau_min          = 0.5;          % minimum headway time                         (s)

% target vehicle trajectory generation
OPTsettings.TVlength         = 4;            % length of the target vehicle                 (m)
OPTsettings.TVinitDist       = 30;           % initial distance of the target vehicle       (m)
OPTsettings.TVinitVel        = 0;            % initial velocity of the target vehicle       (m/s)

OPTsettings.TV_N_hor         = 20;           % horizon length in the TVMPC
OPTsettings.TV_Ts            = 0.5;          % time step used in the TVMPC
OPTsettings.TV_trajEstSett   = 1;            % vehicle distance and velocity estimation method for TVMPC: constant velocity (0), constant acceleration (1), use the previous MPC solutions (2)

OPTsettings.TV_a_LimLowVel  = 1.0;           % acceleration limit of the TVMPC at v < 5 m/s    (m/s^2)
OPTsettings.TV_a_LimHighVel = 0.5;           % acceleration limit of the TVMPC at v > 10 m/s   (m/s^2)
OPTsettings.TV_j_LimLowVel  = 2.0;           % jerk         limit of the TVMPC at v < 5 m/s    (m/s^3)
OPTsettings.TV_j_LimHighVel = 0.5;           % jerk         limit of the TVMPC at v > 10 m/s   (m/s^3)

% stopping profiles
OPTsettings.stopVel          = 0.2;          % velocity at the exact stop position                              (m/s)
OPTsettings.stopRefDist      = 100;          % distance to stopRefvelIncr                                       (m)
OPTsettings.stopRefVelSlope  = 1;            % slope of the velocity limit towards the stop                     (m/s/m)
OPTsettings.TLStopRegionSize = 2;            % size of the low velocity region behind the traffic light         (m)
OPTsettings.TLstopVel        = -1;           % velocity in the low velocity region behind the traffic light     (m)
OPTsettings.alpha_TTL        = 3.34;         % constant for curve negotiation speed constraint (Two-thirds law) (m^(2/3)/s)

% Coefficients of the quadratic and fifth order power consumption fits
OPTsettings.b_quadr      = [185, 1.296e-20, 2.301, 0, 0.003728, -0.000181];
OPTsettings.b_fifthOrder = [185	                     0.427461350854152	 1.33881428409239	    0	...
                             0.00357911242725773	-0.000183195720362408	0	...
                             2.41017657319644e-07	 1.92524236475911e-07	...
                            -1.21266753753542e-08	 2.69420315493954e-12	...
                            -6.73422331977247e-11	-4.65716485427471e-11	...
                            -3.29148609115376e-11	 8.05602619603684e-12	...
                             4.07099563902699e-16	 5.70507905296593e-15	...
                             4.78644673413403e-15	 2.49448524129528e-15	...
                             1.49184094719691e-15	-5.76405750523528e-16];

%% Postprocessing

% move blocking preprocessing
Mb = zeros(1,OPTsettings.N_hor);
j = 1;
for i = 1:length(moveBlockingSettings) 
    n = moveBlockingSettings(i);
    Mb(j:(j+n-1)) = [0, ones(1,n-1)];
    j = j + n;
end
OPTsettings.Mb = Mb;

% Use case
OPTsettings.stopRefvelIncr = OPTsettings.stopRefDist*OPTsettings.stopRefVelSlope;    
OPTsettings = GenerateUseCase(OPTsettings);

end