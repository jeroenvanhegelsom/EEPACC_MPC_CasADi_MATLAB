function V = SetVehicleParameters()
% SetVehicleParameters loads the vehicle parameters
%
%   Inputs:
%       -
%
%   Outputs:
%       V : struct containg all vehicle parameters

    % === VEHICLE ===
        % Vehicle body
    V.m           = 1443;                       % vehicle mass                                	kg
    V.A_f         = 2.38;                       % Frontal area                                  m^2
    V.c_d         = 0.29;                       % Aerodynamic drag coefficient                  -
    V.L           = 2.57;                       % Wheel base length                             m
    V.h_g         = 0.47;                       % Height of the center of gravity               m
    V.WD_s_F      = 0.53;                       % Static weight distribution on  front axle     -
    V.L_f         = V.WD_s_F*V.L;               % Distance between the CoG the front axle       m
    V.L_r         = V.L - V.L_f;                % Static weight distribution on  front axle     m

        % Powertrain
    V.P_m_max     = 125   *1e3;                 % Peak motor power                              kW
    V.T_m_max     = 250;                        % Maximum motor torque                          Nm
    V.omega_m_r   = 4800  /60*2*pi;             % Rated motor speed                             rad/s
    V.omega_m_max = 11400 /60*2*pi;             % Maximum motor speed                           rad/s

        % Transmission
    V.c_r         = 0.0064;                     % Rolling resistance coefficient                -
    V.R_w         = 0.3498;                     % Wheel radius                                  m
    V.beta_gb     = 9.665;                      % Gearbox ratio                                 -
    V.beta_fd     = 1;                          % Final drive gearbox ratio (estimated)         -
    V.phi         = V.beta_gb*V.beta_fd/V.R_w;  % transmission ratio (motor to wheel)           -
        % Battery
    V.U_N         = 353;                        % Nominal battery pack voltage                  V
    V.Q_N         = 94;                         % Nominal battery pack capacity                 V
    V.E_b_gross   = 33.2  *3.6e6;               % Gross battery pack energy content             J
    V.E_b_net     = 27.2  *3.6e6;               % Net battery pack energy content               J

        % Performance
    V.v_max       = 150   /3.6;                 % Top speed                                     m/s
    V.t_acc       = 7.3;                        % Acceleration time (0-100 m/s)                 s
    V.E_v         = 13.1  *3.6e6/1e5;           % Energy consumption                            J/m
    V.D_r         = 300   *1e3;                 % Driving range                                	m

        % Efficiency coefficients
    V.eta_i       = 0.95;                       % Inverter efficiency                           -
    V.eta_gb      = 0.985;                      % Gearbox efficiency                            -
    V.eta_fd      = 0.93;                       % Final drive efficiency                        -
    V.eta_TF      = V.eta_gb*V.eta_fd;          % transmission efficiency (motor to wheel)      -

    % === OTHER ===
    V.lambda      = 1.05;                       % Rotatial inertia inclusion factor             -
    V.P_aux       = 250;                        % Auxilary power usage                          W
    V.mu          = 0.8;                        % 'average' friction coefficient (tire/road)    -
    V.rho_a       = 1.225;                      % Air density (at 15 degrees celsius)           kg/m^3
    V.g           = 9.81;                       % Gravitational acceleration                    m/s^2
    V.zeta_a      = .5*V.c_d*V.rho_a*V.A_f;     % Abbreviation for the constant air             kg/m
                                                    % resistance force terms
end