function [EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot] = InitiateAnimation_MPC(k,s_opt,v_opt,s_opt_pred,v_opt_pred,s_tv_est,v_tv_est,OPTsettings,MPCname)

%% Initialization for the plot

    % construct plot for stops
    if ~isempty(OPTsettings.stopLoc)
        for i = 1:OPTsettings.cutOffDist
            for j = 1:length(OPTsettings.stopLoc)
                distToStop_j = abs(OPTsettings.stopLoc(j) - i);
                maxVStopPlot(j) = distToStop_j*OPTsettings.stopRefVelSlope + OPTsettings.stopVel;
            end
            stopVelConstraintPlot(i) = min(maxVStopPlot);
        end
    else
        stopVelConstraintPlot = 1e5*ones(1,length(1:OPTsettings.cutOffDist));
    end

    % minimum velocity incentive
        % get the minimum of all maximum speeds excluding stops
    [s_velInc,v_velInc] = minPWA(OPTsettings.s_speedLim, OPTsettings.v_speedLim, OPTsettings.s_curv, OPTsettings.alpha_TTL.*(abs(OPTsettings.curvature)).^(-1/3));
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

    %% Figure creation and initial frame

    fh = figure();
        hold on
        lgd = [];
        % speed limit
        plot(OPTsettings.s_speedLim(1:end), 3.6*OPTsettings.v_speedLim(1:end))
            lgd = [lgd,"speed limit"];
        % curve speed
        if max(OPTsettings.curvature)>2e-5
            plot(OPTsettings.s_curv,3.6*OPTsettings.alpha_TTL*(abs(OPTsettings.curvature)).^(-1/3))
            lgd = [lgd,"curve speed policy"];
        end
        % velocity incentive
        plot(s_velInc,3.6*v_velInc,'--')
            lgd = [lgd,"velocity incentive speed"];
        % stop speed profile
        if ~isempty(OPTsettings.stopLoc)
            plot(1:OPTsettings.cutOffDist,3.6*stopVelConstraintPlot)
            lgd = [lgd,"stop constraint profile"];
        end
        % target vehicle (current) position
        TVtrajPlot = plot(s_tv_est(1),3.6*v_tv_est(1),'k.','MarkerSize',10);
            lgd = [lgd,""];
        % target vehicle prediction
        TVpredPlot = plot(s_tv_est,3.6*v_tv_est,'k--');
            lgd = [lgd,"target vehicle"];
        % trajectory
        EVtrajPlot = plot(s_opt(1:k),3.6*v_opt(1:k),'b-');
            lgd = [lgd,join([MPCname,"trajectory"])];
        % prediction
        EVpredPlot = plot(s_opt_pred,3.6*v_opt_pred,'b--');
            lgd = [lgd,"optimized prediction"];
        xlim([0,OPTsettings.cutOffDist])
        ylim([0,3.6*max(OPTsettings.v_speedLim)+5])
        xlabel('$s$ (m)','Interpreter','Latex')
        ylabel('$v$ (km/h)','Interpreter','Latex')
        grid on
        hold off
        legend(lgd,'Interpreter','Latex');
    sdf('LatexFigure');
    fh.WindowState = 'maximized';
    pause
    name = ['C:\Users\20182232\OneDrive - TU Eindhoven\Documents\Courses\Master\Graduation_Project\MATLAB\FINAL_project_files\GIFs\',MPCname,'_UC',num2str(OPTsettings.useCaseNum),'.gif'];
    gif(name,'DelayTime',1/8,'frame',fh)
    
end

