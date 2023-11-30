function [slope_est, v_lim_max,v_stop_max,v_TL_max,v_curv_max,a_min_est,a_max_est,j_min_est,j_max_est] = EstimateRouteAndComfortBounds(OPTsettings,MPCtype,s_est,v_est,t_0,N_hor)
% EstimateRouteAndComfortBounds Estimates travel distance and velocity dependent constraint bounds based on estimates of the trajectory of the ego vehicle
%
%   Inputs:
%       OPTsettings : struct with settings for the optimization
%       MPCtype     : whether it is a FBMPC/ABMPC (0), BLMPC (1) or TVMPC (2)
%       s_est       : estimated travel distance over the horizon
%       v_est       : estimated velocity over the horizon
%       t_0         : initial time for the QP
%       N_hor       : number of steps in the horizon
%
%   Outputs:
%       slope_est  : estimated slopes over the horizon
%       v_lim_max  : estimated speed limits over the horizon
%       v_stop_max : estimated stop forcing speeds over the horizon
%       v_TL_max   : estimated traffic light stop forcing speeds over the horizon
%       v_curv_max : estimated comfortable curve speeeds over the horizon
%       a_min_est  : estimated minimum acceleration over the horizon
%       a_max_est  : estimated maximum acceleration over the horizon
%       j_min_est  : estimated minimum jerk over the horizon
%       j_max_est  : estimated maximum jerk over the horizon

%% Extract settings

    s_speedLim          = OPTsettings.s_speedLim;
    v_speedLim          = OPTsettings.v_speedLim;
    s_curv              = OPTsettings.s_curv;
    curvature           = OPTsettings.curvature;
    s_slope             = OPTsettings.s_slope;
    slope               = OPTsettings.slope;
    stopLoc             = OPTsettings.stopLoc;
    stopRefDist         = OPTsettings.stopRefDist;
    stopRefVelSlope     = OPTsettings.stopRefVelSlope;
    stopVel             = OPTsettings.stopVel;
    TLLoc               = OPTsettings.TLLoc;
    TLstopVel           = OPTsettings.TLstopVel;
    TLStopRegionSize    = OPTsettings.TLStopRegionSize;
    alpha_TTL           = OPTsettings.alpha_TTL;
    Tvec                = OPTsettings.Tvec;

    if MPCtype == 1
        % get BLMPC comfort limits
        BL_a_LimLowVel  = OPTsettings.BL_a_LimLowVel;    
        BL_a_LimHighVel = OPTsettings.BL_a_LimHighVel;  
        BL_j_LimLowVel  = OPTsettings.BL_j_LimLowVel;
        BL_j_LimHighVel = OPTsettings.BL_j_LimHighVel;  
    elseif MPCtype == 2
        % get TVMPC comfort limits
        TV_a_LimLowVel  = OPTsettings.TV_a_LimLowVel;    
        TV_a_LimHighVel = OPTsettings.TV_a_LimHighVel;  
        TV_j_LimLowVel  = OPTsettings.TV_j_LimLowVel;
        TV_j_LimHighVel = OPTsettings.TV_j_LimHighVel;  
    end

    if length(Tvec) < N_hor
        % target vehicle MPC is used
        Tvec = Tvec(1)*ones(1,N_hor);
    end

%% Estimate route based parameters

    % slope
    slope_est = zeros(N_hor,1);
    for i = 1:N_hor
        for j = 1:length(s_slope)
            if j == length(s_slope)
                slope_est(i) = slope(end);
                break;
            elseif s_est(i) >= s_slope(j) && s_est(i) < s_slope(j+1)
                slope_est(i) = slope(j);
                break;
            else
                slope_est(i) = slope(1);
                break;
            end
        end
    end

    % speed limit: get maximum velocity estimates
    v_lim_max = zeros(N_hor,1);
    for i = 1:N_hor
        for j = 1:length(s_speedLim)
            if j == length(s_speedLim)
                v_lim_max(i) = s_speedLim(end);
            elseif s_est(i) >= s_speedLim(j) && s_est(i) < s_speedLim(j+1)
                v_lim_max(i) = v_speedLim(j);
                break;
            end
        end
    end

    % curve: get maximum velocity for comfortable curve negotiation  
    v_curv_max = zeros(N_hor,1);
    for i = 1:N_hor
        for j = 1:length(s_curv)
            if j == length(s_curv)
                v_curv_max(i) = alpha_TTL*(abs(curvature(end)))^(-1/3);
            elseif s_est(i) > s_curv(j) && s_est(i) < s_curv(j + 1)
                v_curv_max(i) = alpha_TTL*(abs(curvature(j)))^(-1/3);
                break;
            end
        end
    end

    % stops: get maximum velocity estimates
    v_stop_max = 1e5*ones(N_hor,1);
    for i = 1:N_hor
        for j = 1:length(stopLoc)
            distToStop_j = abs(stopLoc(j) - s_est(i));
            if distToStop_j < stopRefDist
                v_stop_max(i) = distToStop_j*stopRefVelSlope + stopVel;
            end
        end
    end

    % traffic lights: get maximum velocity estimates
    v_TL_max = 1e5*ones(N_hor,1);
    % check the full horizon
    for i = 1:N_hor
        for j = 1:size(TLLoc,1)
            if mod(t_0 + i*Tvec(i) - TLLoc(j,2), sum(TLLoc(j,3:4))) <  TLLoc(j,3) % check whether the traffic light is red
                distToTL_j = TLLoc(j) - s_est(i);
                if abs(distToTL_j) < stopRefDist
                    if distToTL_j < 0                                       % behind the traffic light
                        v_TL_max(i) = abs(distToTL_j)*stopRefVelSlope + TLstopVel;
                    elseif abs(distToTL_j) < TLStopRegionSize               % close in front of the traffic light
                        v_TL_max(i) = TLstopVel;
                    else                                                    % far in front of the traffic light
                        v_TL_max(i) = abs(distToTL_j-stopVel)*stopRefVelSlope + TLstopVel;
                    end
                end
            end
        end
    end

%% Estimate acceleration and jerk limits

    % Initialize arrays
    a_min_est = zeros(N_hor,1);
    a_max_est = a_min_est;
    j_min_est = a_min_est;
    j_max_est = a_min_est;

    % estimate acceleration and jerk bounds
    for i = 1:N_hor
        if MPCtype == 0
            % ISO limits
            if v_est(i) < 5
                a_min_est(i) = -5;
                a_max_est(i) =  4;
                j_min_est(i) = -5;
                j_max_est(i) =  5;
            elseif v_est(i) < 20
                a_min_est(i) =   -5.5+v_est(i)/10;
                a_max_est(i) = 14/3-2*v_est(i)/15;
                j_min_est(i) =  -35/6+v_est(i)/6;
                j_max_est(i) =   35/6-v_est(i)/6;
            else
                a_min_est(i) = -3.5;
                a_max_est(i) =  2;
                j_min_est(i) = -2.5;
                j_max_est(i) =  2.5;
            end
        elseif MPCtype == 1 % baseline limits
            if v_est(i) < 5
                a_min_est(i) = -BL_a_LimLowVel;
                a_max_est(i) =  BL_a_LimLowVel;
                j_min_est(i) = -BL_j_LimLowVel;
                j_max_est(i) =  BL_j_LimLowVel;
            elseif v_est(i) < 20
                a_min_est(i) =  -( (4*BL_a_LimLowVel-BL_a_LimHighVel)/3  + (BL_a_LimHighVel - BL_a_LimLowVel)/15*v_est(i) );
                a_max_est(i) =     (4*BL_a_LimLowVel-BL_a_LimHighVel)/3  + (BL_a_LimHighVel - BL_a_LimLowVel)/15*v_est(i);
                j_min_est(i) =  -( (4*BL_j_LimLowVel-BL_j_LimHighVel)/3  + (BL_j_LimHighVel - BL_j_LimLowVel)/15*v_est(i) );
                j_max_est(i) =     (4*BL_j_LimLowVel-BL_j_LimHighVel)/3  + (BL_j_LimHighVel - BL_j_LimLowVel)/15*v_est(i);
            else
                a_min_est(i) = -BL_a_LimHighVel;
                a_max_est(i) =  BL_a_LimHighVel;
                j_min_est(i) = -BL_j_LimHighVel;
                j_max_est(i) =  BL_j_LimHighVel;
            end
        else % target vehicle limits
            if v_est(i) < 5
                a_min_est(i) = -TV_a_LimLowVel;
                a_max_est(i) =  TV_a_LimLowVel;
                j_min_est(i) = -TV_j_LimLowVel;
                j_max_est(i) =  TV_j_LimLowVel;
            elseif v_est(i) < 20
                a_min_est(i) =  -( (4*TV_a_LimLowVel-TV_a_LimHighVel)/3  + (TV_a_LimHighVel - TV_a_LimLowVel)/15*v_est(i) );
                a_max_est(i) =     (4*TV_a_LimLowVel-TV_a_LimHighVel)/3  + (TV_a_LimHighVel - TV_a_LimLowVel)/15*v_est(i);
                j_min_est(i) =  -( (4*TV_j_LimLowVel-TV_j_LimHighVel)/3  + (TV_j_LimHighVel - TV_j_LimLowVel)/15*v_est(i) );
                j_max_est(i) =     (4*TV_j_LimLowVel-TV_j_LimHighVel)/3  + (TV_j_LimHighVel - TV_j_LimLowVel)/15*v_est(i);
            else
                a_min_est(i) = -TV_a_LimHighVel;
                a_max_est(i) =  TV_a_LimHighVel;
                j_min_est(i) = -TV_j_LimHighVel;
                j_max_est(i) =  TV_j_LimHighVel;
            end
        end
    end

end

