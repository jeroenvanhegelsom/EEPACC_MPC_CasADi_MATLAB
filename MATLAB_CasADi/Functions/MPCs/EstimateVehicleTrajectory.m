function [s_est, v_est] = EstimateVehicleTrajectory(OPTsettings, estSetting, s_curr, v_curr, a_curr, s_opt_prev_sol, v_opt_prev_sol)
% EstimateVehicleTrajectory Estimates the trajectory of the target or ego vehicle along the prediction horizon
%
%   Inputs:
%       OPTsettings    : struct with settings for the optimization
%       estSetting     : (0) estimate ego vehicle trajectory for FBMPC or ABMPC; (1) estimate target vehicle trajectory; (2) estimation for the TVMPC; (3) estimation for the BLMPC
%       s_curr         : current travel distance for the estimate
%       v_curr         : current velocity for the estimate
%       a_curr         : current acceleration for the estimate
%       s_opt_prev_sol : optimal solution for the travel distance in the horizon of the previous QP
%       v_opt_prev_sol : optimal solution for the velocity in the horizon of the previous QP
%
%   Outputs:
%       s_est          : estimate of the travel distance along the horizon
%       v_est          : estimate of the velocity along the horizon
        
    %% initialization

    % extract settings
    if estSetting == 2
        % for TVMPC
        paramEstSetting = OPTsettings.TV_trajEstSett;
        Ts              = OPTsettings.TV_Ts;
        N_hor           = OPTsettings.TV_N_hor;
    elseif estSetting == 3
        % for BLMPC
        paramEstSetting = OPTsettings.BL_trajEstSett;
        Ts              = OPTsettings.BL_Ts;
        N_hor           = OPTsettings.BL_N_hor;
    else
        % for ABMPC and FBPMC
        N_hor = OPTsettings.N_hor;
        Tvec  = OPTsettings.Tvec;
        if estSetting == 0
            paramEstSetting = OPTsettings.paramEstSetting;
        else
            paramEstSetting = OPTsettings.TVestSetting;
        end
    end

    % const. acceleration time when using paramEstSetting 2
    if estSetting == 1
        tConstACC = OPTsettings.tConstACC_tar;
    else
        tConstACC = OPTsettings.tConstACC_ego;
    end

    % initialize arrays
    s_est = zeros(N_hor+1,1);
    v_est = zeros(N_hor+1,1);

    %% Estimation

    % Estimate distance and velocity for the full horizon
    if paramEstSetting == 0
        % constant velocity
        s_est(1) = s_curr;
        for i = 2:N_hor+1
            if estSetting < 2
                Ts = Tvec(i-1);
            end
            s_est(i) = s_est(i-1) + Ts*v_curr;
        end
        v_est = v_curr*ones(1,N_hor+1);
    elseif paramEstSetting == 1
        % constant acceleration
        s_est(1) = s_curr;
        v_est(1) = v_curr;
        for i = 2:N_hor+1
            if estSetting < 2
                Ts = Tvec(i-1);
            end
            if i <= tConstACC/Ts && v_est(i-1) + Ts*a_curr > 0
                v_est(i) = v_est(i-1) + Ts*a_curr;
            else
                v_est(i) = v_est(i-1);
            end
            s_est(i) = s_est(i-1) + Ts*v_est(i-1);
            
        end
    else
        if estSetting < 2
            Ts = Tvec(end);
        end
        % use the previous MPC solutions
        s_est = [s_curr; s_opt_prev_sol(3:end); s_opt_prev_sol(end)+Ts*v_opt_prev_sol(end)];
        v_est = [v_curr; v_opt_prev_sol(3:end); v_opt_prev_sol(end)];
    end

end