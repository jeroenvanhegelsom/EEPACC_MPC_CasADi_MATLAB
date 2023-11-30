function [T_m,rpm_m,eta_m_fit] = GetMotorEfficiency(useQuadrFit,OPTsettings)
% GetMotorEfficiency_FifthOrder Estimates the efficiency map based on a 
% power consumption map for plotting purposes
%
%   Inputs:
%       useQuadrFit: whether to use fifth order (1) or quadratic (0) fit
%
%   Outputs:
%       T_m   : torques for the efficiency map
%       rpm_m : rpms for the efficiency map
%       eta_m : efficiency values for the efficiency map
%
    %% Initialization
    
    VMP = SetVehicleParameters;          % Fill struct with vehicle parameters
    
    % plotting arrays for the motor speed and torque
    omega_plot = 0:10:round(VMP.omega_m_max);
    T_m_plot = -VMP.T_m_max:VMP.T_m_max;
    
    % mesh grid for the plot
    [T_m,omega_m] = meshgrid(T_m_plot, omega_plot);
    
    % motor speed to rpm
    rpm_m = omega_m*30/pi;
    
    % conversion from motor torque to vehicle traction force
    phi = VMP.beta_gb*VMP.beta_fd/VMP.R_w;
    eta_TF = VMP.eta_gb*VMP.eta_fd;
    F_t = T_m.*phi.*eta_TF.^sign(T_m);
    
    %% Calculate efficiency
    
    % Torque limits T(omega)
    T_m_omega = zeros(1,length(omega_plot));
    for i = 1:length(omega_plot)
        if omega_plot(i) < VMP.omega_m_r
            T_m_omega(i) = VMP.T_m_max;
        else
            T_m_omega(i) = VMP.P_m_max/omega_plot(i);
        end
    end
    
    % get the battery and mechanical power at each data point
    eta_m_fit = zeros(size(T_m));
    for i = 1:size(T_m,1)           % omega index (i)
        for j = 1:size(T_m,2)       % torque index (j)
            if abs(T_m(i,j)) < abs(T_m_omega(i))
                if useQuadrFit
                    P_bat = GetMotorPower_QuadraticFit(F_t(i,j),rpm_m(i,j),OPTsettings.b_quadr)-OPTsettings.b_quadr(1);
                else
                    P_bat = GetMotorPower_FifthOrderSurface(F_t(i,j),rpm_m(i,j),OPTsettings.b_fifthOrder)-OPTsettings.b_fifthOrder(1);
                end
                
                P_mech = T_m(i,j)*omega_m(i,j);

                if T_m(i,j) > 0
                    % propulsion
                    eta_m_fit(i,j) = P_mech./P_bat;
                else
                    % regeneration
                    eta_m_fit(i,j) = P_bat./P_mech;
                end
            end
        end
    end
    
    % efficiency to percentage
    eta_m_fit = eta_m_fit*100;
    
end