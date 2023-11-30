function [s_measured, v_measured] = RunPlantModel(x_0,u,OPTsettings)
% RunPlantModel Nonlinear model to simulate the real plant and get the next state with a given control input. The model uses RK4 with a set number of steps for integration.
%
%   Inputs:
%       x_0         : initial state  := [s_0,v_0]
%       u           : control signal := [F_m,F_b]
%       OPTsettings : struct with settings for the MPC
%
%   Outputs:
%       s_measured : simulated measured travel distance in the next step
%       v_measured : simulated measured velocity in the next step

    %% Initialization

    s_slope = OPTsettings.s_slope;          % PWA slope function distances
    slope   = OPTsettings.slope;            % PWA slope function values
    M       = OPTsettings.N_integratePlant; % number of RK4 steps
    Ts      = OPTsettings.Tvec(1);          % sample time of the controller
    V       = SetVehicleParameters();       % vehicle parameters
    u_k     = sum(u);                       % total force
    DT      = Ts/M;                         % time step per interval
    x_k     = x_0;                          % initial state

    %% Fixed step Runge-Kutta 4 integration
    
    % Run M RK4 integration steps
    for k=1:M
        k1 = f_NL(x_k          , u_k, s_slope, slope);
        k2 = f_NL(x_k + DT/2*k1, u_k, s_slope, slope);
        k3 = f_NL(x_k + DT/2*k2, u_k, s_slope, slope);
        k4 = f_NL(x_k + DT  *k3, u_k, s_slope, slope);
        x_k = x_k + DT/6*(k1 + 2*k2 + 2*k3 + k4);
    end

    % set the measurement simulation result
    s_measured = x_k(1);
    v_measured = x_k(2);
    
    %% Function containing the nonlinear dynamics

    function x_dot = f_NL(x_k, u_k, s_slope, slope)
        theta = InterpPWA(x_k(1),s_slope,slope);
        x_dot = [x_k(2), 1/V.lambda/V.m*(u_k - V.zeta_a*x_k(2)^2 - V.c_r*V.m*V.g*cos(theta) - V.m*V.g*sin(theta))];
    end

end

