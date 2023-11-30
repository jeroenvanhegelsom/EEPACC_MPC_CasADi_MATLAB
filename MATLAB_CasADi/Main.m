clearvars;clc;close all;
addpath('.\casadi_3_6_3')
V = SetVehicleParameters();

%% General settings

% Which solutions to recalculate
recalculateNLP   = true;
recalculateFBMPC = true;
recalculateABMPC = true;
recalculateBLMPC = true;

% which solutions to plot
plotNLP   = false;
plotFBMPC = true;
plotABMPC = true;
plotBLMPC = true;

% === other settings ===
% whether to plot additional plots (comfort, calculation times, headway, etc.)
createAdditionalPlots  = true;
% whether to create gifs of the receding horizon process of the MPCs
OPTsettings.createGifs = false;
% whether to include a target vehicle (generated if not in use case)
OPTsettings.IncludeTV  = false;
% whether to use the simulation time from a use case instead of the one set below
useUseCaseTsim         = true;

% == use cases ==
%  1: accelerating from standstill to 80 km/h
%  2: braking to standstill from 80 km/h
%  3: several changes in the speed limit
%  4: approaching a stop
%  5: driving a route with several traffic lights
%  6: approaching and driving through several curves
%  7: climbing and descending a hill
%  8: driving behind an aggressive target vehicle
%  9: driving behind a less aggressive target vehicle
% 10: target vehicle cutting into the lane
% 11: long route scenario
% 12: scenario for the video
OPTsettings.useCaseNum = 8;


% time step and simulation time
OPTsettings.Ts     = 0.5;    % controller sample time (s) (MUST BE a multiple of .1 seconds!)
if ~useUseCaseTsim
    % if a custom simulation time is used
    OPTsettings.t_sim  = 60; % total simulation time  (s)
end

%% Processing

% Get other settings
OPTsettings = Settings(OPTsettings);
cutOffDist = OPTsettings.cutOffDist;

if OPTsettings.useCaseNum == 0 % custom use case
    % if TV is requested, it must be generated
    OPTsettings.generateTVMPC = true;
end

% transpose the target vehicle vectors if required
if isfield(OPTsettings,'s_tv')
    if size(OPTsettings.s_tv,1) < size(OPTsettings.s_tv,2) 
        OPTsettings.s_tv = OPTsettings.s_tv';
        OPTsettings.v_tv = OPTsettings.v_tv';
    end
end

% Target vehicle trajectory generation
if ~OPTsettings.IncludeTV
    % no target vehicle: s=inf, v=0
    if ~isfield(OPTsettings,'s_tv') || isempty(OPTsettings.s_tv)
        OPTsettings.s_tv = inf*ones(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
        OPTsettings.v_tv =    zeros(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
    end
else
    if OPTsettings.generateTVMPC
        % use an MPC to generate the target vehicle's trajectory
        fprintf ('=== GENERATING TARGET VEHICLE TRAJECTORY ===\n\n');
        [OPTsettings.s_tv, OPTsettings.v_tv, TVbadExitMessages] = RunOpt_TVMPC(OPTsettings);
        % shift to the back of the vehicle
        OPTsettings.s_tv = OPTsettings.s_tv - OPTsettings.TVlength;
    end
end

%% Run simulation(s)

% NLP benchmark
if recalculateNLP
    fprintf ('\n=== SOLVING NLP-BASED OCP ===\n\n');
    NLPsol  = RunOpt_NLP(OPTsettings);   
    save('savedNLPsol.mat','NLPsol');
else
    NLPsol = load('savedNLPsol.mat').NLPsol;
end

% Force-based MPC
if recalculateFBMPC
    fprintf ('\n=== SOLVING FORCE-BASED MPC ===\n\n');
    FBMPCsol = RunOpt_FBMPC(OPTsettings);
    save('savedFBMPCsol.mat','FBMPCsol');
else
    FBMPCsol = load('savedFBMPCsol.mat').FBMPCsol;
end

% Acceleration-based MPC
if recalculateABMPC
    fprintf ('\n=== SOLVING ACCELERATION-BASED MPC ===\n\n');
    ABMPCsol = RunOpt_ABMPC(OPTsettings);
    save('savedABMPCsol.mat','ABMPCsol');
else
    ABMPCsol = load('savedABMPCsol.mat').ABMPCsol;
end

% Baseline MPC
if recalculateBLMPC
    fprintf ('\n=== SOLVING BASELINE MPC ===\n\n');
    BLMPCsol = RunOpt_BLMPC(OPTsettings);
    save('savedBLMPCsol.mat','BLMPCsol');
else
    BLMPCsol = load('savedBLMPCsol.mat').BLMPCsol;
end

%% Show results

% find if the target vehicle is used
maxVel = max([FBMPCsol.v_opt;      NLPsol.v_opt;      ABMPCsol.v_opt;      BLMPCsol.v_opt]);
vLimAtCutOffDist = InterpPWA(cutOffDist,OPTsettings.s_speedLim,OPTsettings.v_speedLim);

% cutting off the trajectories around the minimum distance for fair comparisons
distIndFBMPC= 0;
for i = 1:length(FBMPCsol.s_opt)-1

    if FBMPCsol.s_opt(i) < cutOffDist && FBMPCsol.s_opt(i+1) > cutOffDist
        distIndFBMPC = i;
        break;
    end

end
if distIndFBMPC == 0
    distIndFBMPC = length(FBMPCsol.s_opt)-1;
end

distIndABMPC= 0;
for i = 1:length(ABMPCsol.s_opt)-1

    if ABMPCsol.s_opt(i) < cutOffDist && ABMPCsol.s_opt(i+1) > cutOffDist
        distIndABMPC = i;
        break;
    end

end
if distIndABMPC == 0
    distIndABMPC = length(ABMPCsol.s_opt)-1;
end

distIndBLMPC= 0;
for i = 1:length(BLMPCsol.s_opt)-1

    if BLMPCsol.s_opt(i) < cutOffDist && BLMPCsol.s_opt(i+1) > cutOffDist
        distIndBLMPC = i;
        break;
    end

end
if distIndBLMPC == 0
    distIndBLMPC = length(BLMPCsol.s_opt)-1;
end

distIndNLP= 0;
for i = 1:length(NLPsol.s_opt)-1

    if NLPsol.s_opt(i) < cutOffDist && NLPsol.s_opt(i+1) > cutOffDist
        distIndNLP = i;
        break;
    end

end
if distIndNLP == 0
    distIndNLP = length(NLPsol.s_opt)-1;
elseif distIndNLP

end

% print results at t=t_sim
fprintf('\n');
fprintf(['=== RESULTS of UC',num2str(OPTsettings.useCaseNum),' ===\n']);
fprintf('\n');
if OPTsettings.solverToUse == 0
    fprintf('MPC solver: sparse qpOASES\n');
elseif OPTsettings.solverToUse == 1
    fprintf('MPC solver: dense qpOASES\n');
else
    fprintf('MPC solver: sparse HPIPM\n');
end
fprintf('\n');
fprintf('Feasibility:\n')
if OPTsettings.generateTVMPC && OPTsettings.IncludeTV
    fprintf (['   TVMPC: ',num2str(TVbadExitMessages),' bad exit messages\n']);
end
fprintf (['   NLP:   ',NLPsol.exitMessage,'\n']);
fprintf (['   FBMPC: ',num2str(sum(FBMPCsol.exitMessage)),' bad exit messages\n']);
fprintf (['   ABMPC: ',num2str(sum(ABMPCsol.exitMessage)),' bad exit messages\n']);
fprintf (['   BLMPC: ',num2str(sum(BLMPCsol.exitMessage)),' bad exit messages\n']);
fprintf('\n');
fprintf('MPC loop times:\n')
fprintf (['   Force-based MPC:        rms. time = ',num2str(rms(FBMPCsol.tSolve(1:distIndFBMPC))*1e3),' ms, std = ',num2str(std(FBMPCsol.tSolve(1:distIndFBMPC))*1e3),' ms, max = ',num2str(max(FBMPCsol.tSolve(1:distIndFBMPC))*1e3),' ms \n']);
fprintf (['   Acceleration-based MPC: rms. time = ',num2str(rms(ABMPCsol.tSolve(1:distIndABMPC))*1e3),' ms, std = ',num2str(std(ABMPCsol.tSolve(1:distIndABMPC))*1e3),' ms, max = ',num2str(max(ABMPCsol.tSolve(1:distIndABMPC))*1e3),' ms \n']);
fprintf (['   Baseline MPC:           rms. time = ',num2str(rms(BLMPCsol.tSolve(1:distIndBLMPC))*1e3),' ms, std = ',num2str(std(BLMPCsol.tSolve(1:distIndBLMPC))*1e3),' ms, max = ',num2str(max(BLMPCsol.tSolve(1:distIndBLMPC))*1e3),' ms \n']);
fprintf('\n');
fprintf('Distance traveled:\n')
fprintf (['   Force-based MPC:         ',num2str(FBMPCsol.s_opt(end)/1e3),' km\n']);
fprintf (['   Acceleration-based MPC:  ',num2str(ABMPCsol.s_opt(end)/1e3),' km\n']);
fprintf (['   Baseline MPC:            ',num2str(BLMPCsol.s_opt(end)/1e3),' km\n']);
fprintf (['   NLP:                     ',num2str(  NLPsol.s_opt(end)/1e3),' km\n']);
fprintf('\n');
fprintf('Energy consumption:\n')
fprintf (['   Force-based MPC:        ',num2str(FBMPCsol.E_opt(end)/3.6e6),' kWh\n']);
fprintf (['   Acceleration-based MPC: ',num2str(ABMPCsol.E_opt(end)/3.6e6),' kWh\n']);
fprintf (['   Baseline MPC:           ',num2str(BLMPCsol.E_opt(end)/3.6e6),' kWh\n']);
fprintf (['   NLP:                    ',num2str(  NLPsol.E_opt(end)/3.6e6),' kWh\n']);
fprintf('\n');
fprintf(['Error relative to the speed limit at ',num2str(cutOffDist/1e3),' km:\n']);
fprintf (['   Force-based MPC:        ',num2str(vLimAtCutOffDist - FBMPCsol.v_opt(distIndFBMPC-1)),' m/s\n']);
fprintf (['   Acceleration-based MPC: ',num2str(vLimAtCutOffDist - ABMPCsol.v_opt(distIndABMPC-1)),' m/s\n']);
fprintf (['   Baseline MPC:           ',num2str(vLimAtCutOffDist - BLMPCsol.v_opt(distIndBLMPC-1)),' m/s\n']);
fprintf (['   NLP:                    ',num2str(vLimAtCutOffDist -   NLPsol.v_opt(distIndNLP-1)),  ' m/s\n']);
fprintf('\n');

% get the cut-off time
t_sCutOff_FBMPC = .1*round(distIndFBMPC*OPTsettings.Tvec(1)*10);
t_sCutOff_ABMPC = .1*round(distIndABMPC*OPTsettings.Tvec(1)*10);
t_sCutOff_BLMPC = .1*round(distIndBLMPC*OPTsettings.Tvec(1)*10);
t_sCutOff_NLP   = .1*round(distIndNLP*OPTsettings.Tvec(1)*10);

% energy, travel time and comfort at the cut-off distance
fprintf(['Energy consumption at ',num2str(cutOffDist/1e3),' km:\n']);
fprintf (['   Force-based MPC:        ',num2str(FBMPCsol.E_opt(distIndFBMPC-1)/3.6e6),' kWh\n']);
fprintf (['   Acceleration-based MPC: ',num2str(ABMPCsol.E_opt(distIndABMPC-1)/3.6e6),' kWh\n']);
fprintf (['   Baseline MPC:           ',num2str(BLMPCsol.E_opt(distIndBLMPC-1)/3.6e6),' kWh\n']);
fprintf (['   NLP:                    ',num2str(  NLPsol.E_opt(distIndNLP-1)/3.6e6),  ' kWh\n']);
fprintf('\n');
fprintf(['Travel time at ',num2str(cutOffDist/1e3),' km:\n']);
fprintf (['   Force-based MPC:        ',num2str(t_sCutOff_FBMPC),' s\n']);
fprintf (['   Acceleration-based MPC: ',num2str(t_sCutOff_ABMPC),' s\n']);
fprintf (['   Baseline MPC:           ',num2str(t_sCutOff_BLMPC),' s\n']);
fprintf (['   NLP:                    ',num2str(t_sCutOff_NLP),' s\n']);
fprintf('\n');
fprintf(['Comfort metrics at ',num2str(cutOffDist/1e3),' km:\n']);
fprintf (['   Force-based MPC:        max. a = ',num2str(max(FBMPCsol.a_opt(1:distIndFBMPC))),'m/s2, min. a = ',num2str(min(FBMPCsol.a_opt(1:distIndFBMPC))),'m/s2, rms. a = ',num2str(rms(FBMPCsol.a_opt(1:distIndFBMPC))),'m/s2\n'])
fprintf (['   Acceleration-based MPC: max. a = ',num2str(max(ABMPCsol.a_opt(1:distIndABMPC))),'m/s2, min. a = ',num2str(min(ABMPCsol.a_opt(1:distIndABMPC))),'m/s2, rms. a = ',num2str(rms(ABMPCsol.a_opt(1:distIndABMPC))),'m/s2\n'])
fprintf (['   Baseline MPC:           max. a = ',num2str(max(BLMPCsol.a_opt(1:distIndBLMPC))),'m/s2, min. a = ',num2str(min(BLMPCsol.a_opt(1:distIndBLMPC))),'m/s2, rms. a = ',num2str(rms(BLMPCsol.a_opt(1:distIndBLMPC))),'m/s2\n'])
fprintf('\n');
fprintf (['   Force-based MPC:        max. j = ',num2str(max(FBMPCsol.j_opt(1:distIndFBMPC))),'m/s3, min. j = ',num2str(min(FBMPCsol.j_opt(1:distIndFBMPC))),'m/s3, rms. j = ',num2str(rms(FBMPCsol.j_opt(1:distIndFBMPC))),'m/s3\n'])
fprintf (['   Acceleration-based MPC: max. j = ',num2str(max(ABMPCsol.j_opt(1:distIndABMPC))),'m/s3, min. j = ',num2str(min(ABMPCsol.j_opt(1:distIndABMPC))),'m/s3, rms. j = ',num2str(rms(ABMPCsol.j_opt(1:distIndABMPC))),'m/s3\n'])
fprintf (['   Baseline MPC:           max. j = ',num2str(max(BLMPCsol.j_opt(1:distIndBLMPC))),'m/s3, min. j = ',num2str(min(BLMPCsol.j_opt(1:distIndBLMPC))),'m/s3, rms. j = ',num2str(rms(BLMPCsol.j_opt(1:distIndBLMPC))),'m/s3\n'])
fprintf('\n');

%% Get data

% === Minimum velocity incentive ===
    
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

%% Plot solutions

Ts = OPTsettings.Tvec(1);
plotVehicleVollowingPlots = isfield(OPTsettings,'s_tv') && (sum(OPTsettings.s_tv < 1e6) > 1);
t_plot    = 0:Ts:OPTsettings.t_sim;
t_plot_TV = t_plot;

% states and control signals over time
figure()
subplot(2,1,1)
    % distance
    yyaxis left
    lgd = [];
    hold on
    if plotFBMPC
        plot(t_plot,FBMPCsol.s_opt)
        lgd = [lgd,"$s$ (FBMPC)"];
    end
    if plotABMPC
        plot(t_plot,ABMPCsol.s_opt)
        lgd = [lgd,"$s$ (ABMPC)"];
    end
    if plotBLMPC
        plot(t_plot,BLMPCsol.s_opt)
        lgd = [lgd,"$s$ (BLMPC)"];
    end
    if plotNLP
        plot(t_plot,NLPsol.s_opt)
        lgd = [lgd,"$s$ (NLP)"];
    end
    ylabel('$s$ (m)','Interpreter','Latex')
    hold on
    % velocity
    yyaxis right
    hold on
    if plotFBMPC
        plot(t_plot,FBMPCsol.v_opt)
        lgd = [lgd,"$v$ (FBMPC)"];
    end
    if plotABMPC
        plot(t_plot,ABMPCsol.v_opt)
        lgd = [lgd,"$v$ (ABMPC)"];
    end
    if plotBLMPC
        plot(t_plot,BLMPCsol.v_opt)
        lgd = [lgd,"$v$ (BLMPC)"];
    end
    if plotNLP
        plot(t_plot,NLPsol.v_opt)
        lgd = [lgd,"$v$ (NLP)"];
    end
    ylabel('$v$ (m/s)','Interpreter','Latex')
    ylim([0,maxVel+2])
    xlim([0,cutOffDist])
    grid on
    legend(lgd,'Interpreter','Latex')
subplot(2,1,2)
    lgd = [];
    hold on
    plot(0,0,'k-')
    plot(0,0,'k--')
        lgd = [lgd,"$F_m$","$F_b$"];
    if plotFBMPC
        stairs(t_plot,[FBMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0      0.4470 0.7410])
        stairs(t_plot,[FBMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0      0.4470 0.7410])
    end
    if plotABMPC
        stairs(t_plot,[ABMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0.8500 0.3250 0.0980])
        stairs(t_plot,[ABMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0.8500 0.3250 0.0980])
    end
    if plotBLMPC
        stairs(t_plot,[BLMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0.9290 0.6940 0.1250])
        stairs(t_plot,[BLMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0.9290 0.6940 0.1250])
    end
    if plotNLP
        stairs(t_plot,  [  NLPsol.Fm_opt(1:end);   nan],'-', 'Color',[0.4940 0.1840 0.5560])
        stairs(t_plot,  [  NLPsol.Fb_opt(1:end);   nan],'--','Color',[0.4940 0.1840 0.5560])
    end
    grid on
    xlabel('$t$ (s)','Interpreter','Latex')
    xlim([0,cutOffDist])
    ylabel('$F$ (N)','Interpreter','Latex')
    legend(lgd,'Interpreter','Latex')
sdf('LatexFigure');

% construct plot for stops
if ~isempty(OPTsettings.stopLoc)
    for i = 1:cutOffDist
        for j = 1:length(OPTsettings.stopLoc)
            distToStop_j = abs(OPTsettings.stopLoc(j) - i);
            maxVStopPlot(j) = distToStop_j*OPTsettings.stopRefVelSlope + OPTsettings.stopVel;
        end
        stopVelConstraintPlot(i) = min(maxVStopPlot);
    end
else
    stopVelConstraintPlot = 1e5*ones(1,length(1:cutOffDist));
end

% velocity and control signals over distance with distance-based
% constraints visualized and energy consumption
figure()
subplot(3,1,1)
    lgd = [];
    hold on
    if plotFBMPC
        plot(FBMPCsol.s_opt,FBMPCsol.v_opt)
        lgd = [lgd,join(["FBMPC ($",num2str(t_sCutOff_FBMPC),"$ s)"])];
    end
    if plotABMPC
        plot(ABMPCsol.s_opt,ABMPCsol.v_opt)
        lgd = [lgd,join(["ABMPC ($",num2str(t_sCutOff_ABMPC),"$ s)"])];
    end
    if plotBLMPC
        plot(BLMPCsol.s_opt,BLMPCsol.v_opt)
        lgd = [lgd,join(["BLMPC ($",num2str(t_sCutOff_BLMPC),"$ s)"])];
    end
    if plotNLP
        plot(NLPsol.s_opt,NLPsol.v_opt)
        lgd = [lgd,join(["NLP ($",num2str(t_sCutOff_NLP),"$ s)"])];
    end
    plot(OPTsettings.s_speedLim(1:end), OPTsettings.v_speedLim(1:end))
        lgd = [lgd,"speed limit"];
    if max(OPTsettings.curvature)>2e-5
        plot(OPTsettings.s_curv,OPTsettings.alpha_TTL*(abs(OPTsettings.curvature)).^(-1/3))
        lgd = [lgd,"curve speed policy"];
    end
    plot(s_velInc,v_velInc,'--')
        lgd = [lgd,"velocity incentive speed"];
    if ~isempty(OPTsettings.stopLoc)
        plot(1:cutOffDist,stopVelConstraintPlot)
        lgd = [lgd,"stop constraint profile"];
    end
    if size(OPTsettings.TLLoc,1) > 0
        for i = 1:size(OPTsettings.TLLoc,1)
            xline(OPTsettings.TLLoc(i,1),'r--')
        end
        lgd = [lgd,"stop constraint profile"];
    end
    legend(lgd,'Interpreter','Latex')
    ylabel('$v$ (m/s)','Interpreter','Latex')
    ylim([0, maxVel+2])
    xlim([0, cutOffDist])
    grid on
subplot(3,1,2)
    lgd = [];
    hold on
    plot(0,0,'k-')
    plot(0,0,'k--')
        lgd = [lgd,"$F_m$","$F_b$"];
    if plotFBMPC
        stairs(FBMPCsol.s_opt,[FBMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0      0.4470 0.7410])
        stairs(FBMPCsol.s_opt,[FBMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0      0.4470 0.7410])
    end
    if plotABMPC
        stairs(ABMPCsol.s_opt,[ABMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0.8500 0.3250 0.0980])
        stairs(ABMPCsol.s_opt,[ABMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0.8500 0.3250 0.0980])
    end
    if plotBLMPC
        stairs(BLMPCsol.s_opt,[BLMPCsol.Fm_opt(1:end-1); nan],'-', 'Color',[0.9290 0.6940 0.1250])
        stairs(BLMPCsol.s_opt,[BLMPCsol.Fb_opt(1:end-1); nan],'--','Color',[0.9290 0.6940 0.1250])
    end
    if plotNLP
        stairs(NLPsol.s_opt,  [  NLPsol.Fm_opt(1:end);   nan],'-', 'Color',[0.4940 0.1840 0.5560])
        stairs(NLPsol.s_opt,  [  NLPsol.Fb_opt(1:end);   nan],'--','Color',[0.4940 0.1840 0.5560])
    end
    grid on
    ylabel('$F$ (N)','Interpreter','Latex')
    xlim([0,cutOffDist])
    legend(lgd,'Interpreter','Latex')
subplot(3,1,3)
    lgd = [];
    hold on;
    if plotFBMPC
        plot(FBMPCsol.s_opt,FBMPCsol.E_opt/3.6e6)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.s_opt,ABMPCsol.E_opt/3.6e6)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.s_opt,BLMPCsol.E_opt/3.6e6)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.s_opt(1:end-1),NLPsol.E_opt/3.6e6)
        lgd = [lgd,"NLP"];
    end
    grid on
    legend(lgd,'Interpreter','Latex')
    ylabel('$E_\textrm{tot}$ (kWh)','Interpreter','Latex')
    xlim([0,cutOffDist])
    xlabel('$s$ (m)','Interpreter','Latex')
sdf('LatexFigure');

% Distance over time
figure()
lgd = [];
hold on
if plotFBMPC
    plot(t_plot,FBMPCsol.s_opt)
    lgd = [lgd,"FBMPC"];
end
if plotABMPC
    plot(t_plot,ABMPCsol.s_opt)
    lgd = [lgd,"ABMPC"];
end
if plotBLMPC
    plot(t_plot,BLMPCsol.s_opt)
    lgd = [lgd,"BLMPC"];
end
if plotNLP
    plot(t_plot,NLPsol.s_opt)
    lgd = [lgd,"NLP"];
end
% target vehicle
if plotVehicleVollowingPlots 
    plot(t_plot_TV,OPTsettings.s_tv,'k--')
    lgd = [lgd,"target vehicle"];
end
% stops
if ~isempty(OPTsettings.stopLoc)
    for i = 1:length(OPTsettings.stopLoc)
        yline(OPTsettings.stopLoc(i),'k:');
    end
    lgd = [lgd,"stop"];
end
% traffic lights
if size(OPTsettings.TLLoc,1)>0
    for i = 1:size(OPTsettings.TLLoc,1)
        for j = 0:ceil(OPTsettings.t_sim/sum(OPTsettings.TLLoc(i,3:4))+1)
            TLplot_t = [OPTsettings.TLLoc(i,2) + j*sum(OPTsettings.TLLoc(i,3:4)), OPTsettings.TLLoc(i,2) + j*sum(OPTsettings.TLLoc(i,3:4)) + OPTsettings.TLLoc(i,3)];
            TLplot_d = ones(1,2)*OPTsettings.TLLoc(i);
            plot(TLplot_t, TLplot_d,'r');
        end
    end
    lgd = [lgd,"red traffic lights"];
end
legend(lgd,'Interpreter','Latex')
xlabel('$t$ (s)','Interpreter','Latex')
ylabel('$s$ (m)','Interpreter','Latex')
ylim([0, cutOffDist])
xlim([0, OPTsettings.t_sim])
grid on
sdf('LatexFigure');

if createAdditionalPlots
    
    % acceleration and jerk over time
    figure()
    subplot(2,1,1)
        lgd = [];
        hold on;
        if plotFBMPC
            plot(t_plot,FBMPCsol.a_opt)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot,ABMPCsol.a_opt)
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            plot(t_plot,BLMPCsol.a_opt)
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            plot(t_plot(2:end),NLPsol.a_opt)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel(['$a$ (m s$^{-2}$)'],'Interpreter','Latex')
    subplot(2,1,2)
        lgd = [];
                hold on;
        if plotFBMPC
            plot(t_plot(2:end),FBMPCsol.j_opt)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(2:end),ABMPCsol.j_opt)
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            plot(t_plot(2:end),BLMPCsol.j_opt)
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            plot(t_plot,NLPsol.j_opt)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel(['$j$ (m s$^{-3}$)'],'Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % Filtered acceleration and jerk over time
    figure()
    filterSize = 3; % steps (should be odd)
    subplot(2,1,1)
        lgd = [];
        hold on;
        if plotFBMPC
            filterRange = (filterSize-1):(length(FBMPCsol.a_opt)-(filterSize-1));
            a_movingAvg = zeros(length(FBMPCsol.a_opt),1);
            for i = filterRange
                a_movingAvg(i) = sum(FBMPCsol.a_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot,a_movingAvg)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            filterRange = (filterSize-1):(length(ABMPCsol.a_opt)-(filterSize-1));
            a_movingAvg = zeros(length(ABMPCsol.a_opt),1);
            for i = filterRange
                a_movingAvg(i) = sum(ABMPCsol.a_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot,a_movingAvg)
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            filterRange = (filterSize-1):(length(BLMPCsol.a_opt)-(filterSize-1));
            a_movingAvg = zeros(length(BLMPCsol.a_opt),1);
            for i = filterRange
                a_movingAvg(i) = sum(BLMPCsol.a_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot,a_movingAvg)
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            filterRange = (filterSize-1):(length(NLPsol.a_opt)-(filterSize-1));
            a_movingAvg = zeros(length(NLPsol.a_opt)+1,1);
            for i = filterRange
                a_movingAvg(i) = sum(NLPsol.a_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot,a_movingAvg)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel(['$a_\textrm{avg,',num2str(round(10*filterSize*Ts)/10),' s}$ (m s$^{-2}$)'],'Interpreter','Latex')
    subplot(2,1,2)
        lgd = [];
        hold on;
        if plotFBMPC
            filterRange = (filterSize-1):(length(FBMPCsol.j_opt)-(filterSize-1));
            j_movingAvg = zeros(length(FBMPCsol.j_opt),1);
            for i = filterRange
                j_movingAvg(i) = sum(FBMPCsol.j_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot(2:end),j_movingAvg)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            filterRange = (filterSize-1):(length(ABMPCsol.j_opt)-(filterSize-1));
            j_movingAvg = zeros(length(ABMPCsol.j_opt),1);
            for i = filterRange
                j_movingAvg(i) = sum(ABMPCsol.j_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot(2:end),j_movingAvg)
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            filterRange = (filterSize-1):(length(BLMPCsol.j_opt)-(filterSize-1));
            j_movingAvg = zeros(length(BLMPCsol.j_opt),1);
            for i = filterRange
                j_movingAvg(i) = sum(BLMPCsol.j_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot(2:end),j_movingAvg)
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            filterRange = (filterSize-1):(length(NLPsol.j_opt)-(filterSize-1));
            j_movingAvg = zeros(length(NLPsol.j_opt),1);
            for i = filterRange
                j_movingAvg(i) = sum(NLPsol.j_opt(i-(filterSize-1)/2:i+(filterSize-1)/2))/filterSize;
            end
            plot(t_plot,j_movingAvg)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel(['$j_\textrm{avg,',num2str(round(10*filterSize*Ts)/10),' s}$ (m s$^{-3}$)'],'Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    
    if plotVehicleVollowingPlots
        
        % Headway distance over velocity
            A_hwp = 2;
            T_hwp = 2;
            G_hwp = -0.0246*T_hwp + 0.010819;
            v_pol = 0:maxVel;
            h_pol = A_hwp + T_hwp*v_pol + G_hwp*v_pol.^2;
            h_min = max(ones(1,length(v_pol))*OPTsettings.h_min,v_pol*OPTsettings.tau_min);
        figure()
        lgd = [];
        hold on
        if plotFBMPC
            plot(FBMPCsol.v_opt, OPTsettings.s_tv - FBMPCsol.s_opt,'-*')
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(ABMPCsol.v_opt, OPTsettings.s_tv - ABMPCsol.s_opt,'-*')
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            plot(BLMPCsol.v_opt, OPTsettings.s_tv - BLMPCsol.s_opt,'-*')
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            plot(FBMPCsol.v_opt, OPTsettings.s_tv - NLPsol.s_opt,'-*')
            lgd = [lgd,"NLP"];
        end
        hold on
        plot(v_pol,h_pol,'k--')
            lgd = [lgd,"desired headway distance policy",];
        plot(v_pol,h_min,'k')
            lgd = [lgd,"minimum headway distance",];
        xlim([0, maxVel])
        ylim([0,100])
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel('$h$ (m)','Interpreter','Latex')
        xlabel('$v$ (m/s)','Interpreter','Latex')
        sdf('LatexFigure');
        
        % Headway distance over time
        figure()
        lgd = [];
        hold on
        if plotFBMPC
            plot(t_plot, OPTsettings.s_tv - FBMPCsol.s_opt,'-*')
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot, OPTsettings.s_tv - ABMPCsol.s_opt,'-*')
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            plot(t_plot, OPTsettings.s_tv - BLMPCsol.s_opt,'-*')
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            plot(t_plot, OPTsettings.s_tv - NLPsol.s_opt,'-*')
            lgd = [lgd,"NLP"];
        end
        grid on
        ylim([0,100])
        legend(lgd,'Interpreter','Latex')
        ylabel('$h$ (m)','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
        sdf('LatexFigure');
        
        % Headway time over time
        figure()
        lgd = [];
        hold on
        if plotFBMPC
            plot(t_plot,(OPTsettings.s_tv - FBMPCsol.s_opt)./FBMPCsol.v_opt)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot,(OPTsettings.s_tv - ABMPCsol.s_opt)./ABMPCsol.v_opt)
            lgd = [lgd,"ABMPC"];
        end
        if plotBLMPC
            plot(t_plot,(OPTsettings.s_tv - BLMPCsol.s_opt)./BLMPCsol.v_opt)
            lgd = [lgd,"BLMPC"];
        end
        if plotNLP
            plot(t_plot,(OPTsettings.s_tv - NLPsol.s_opt)./NLPsol.v_opt)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel('$\tau$ (s)','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
        sdf('LatexFigure');
    
    end
    
    % Energy consumption over time
    figure()
    lgd = [];
    hold on;
    if plotFBMPC
        plot(t_plot,FBMPCsol.E_opt/3.6e6)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(t_plot,ABMPCsol.E_opt/3.6e6)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(t_plot,BLMPCsol.E_opt/3.6e6)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(t_plot(1:end-1),NLPsol.E_opt/3.6e6)
        lgd = [lgd,"NLP"];
    end
    grid on
    legend(lgd,'Interpreter','Latex')
    ylabel('$E_\textrm{cumulative}$ (kWh)','Interpreter','Latex')
    xlabel('$t$ (s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % Energy consumption over distance
    figure()
    lgd = [];
    hold on;
    if plotFBMPC
        plot(FBMPCsol.s_opt,FBMPCsol.E_opt/3.6e6)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.s_opt,ABMPCsol.E_opt/3.6e6)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.s_opt,BLMPCsol.E_opt/3.6e6)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.s_opt(1:end-1),NLPsol.E_opt/3.6e6)
        lgd = [lgd,"NLP"];
    end
    grid on
    legend(lgd,'Interpreter','Latex')
    ylabel('$E_\textrm{cumulative}$ (kWh)','Interpreter','Latex')
    xlabel('$s$ (m)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % MPC loop times
    figure()
    lgd = [];
    hold on;
    if plotFBMPC
        plot(FBMPCsol.tLoop*1e3,'-*')
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.tLoop*1e3,'-*')
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.tLoop*1e3,'-*')
        lgd = [lgd,"BLMPC"];
    end
    grid on
    ylim([0,100])
    xlim([0,length(FBMPCsol.tLoop)])
    legend(lgd,'Interpreter','Latex')
    ylabel('$t_\textrm{loop}$ (ms)','Interpreter','Latex')
    xlabel('$k$','Interpreter','Latex')
    sdf('LatexFigure');
    
    % MPC solving times
    figure()
    lgd = [];
    hold on
    if plotFBMPC
        plot(t_plot(1:distIndFBMPC),FBMPCsol.tSolve(1:distIndFBMPC)*1e3,'-*')
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(t_plot(1:distIndABMPC),ABMPCsol.tSolve(1:distIndABMPC)*1e3,'-*')
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(t_plot(1:distIndBLMPC),BLMPCsol.tSolve(1:distIndBLMPC)*1e3,'-*')
        lgd = [lgd,"BLMPC"];
    end
    grid on
    ylim([0,100])
    xlim([0,OPTsettings.t_sim/OPTsettings.Tvec(1)+1])
    legend(lgd,'Interpreter','Latex')
    ylabel('$t_\textrm{solve}$ (ms)','Interpreter','Latex')
    xlabel('$t$ (s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % determine motor torque limits
    rpm_max = V.omega_m_max*30/pi;
    T_m_omega = zeros(1,floor(rpm_max));
    for i = 1:floor(rpm_max)
        if i < V.omega_m_r*(30/pi)
            T_m_omega(i) = V.T_m_max;
        else
            T_m_omega(i) = V.P_m_max/(i*pi/30);
        end
    end
    rpm_plot = 1:floor(rpm_max);
    
    % get efficiency map
    [T_m_eff,rpm_m_eff,eta_m_fit_eff] = GetMotorEfficiency(false,OPTsettings);
    
    if distIndNLP > length(NLPsol.rpm_opt)
        distIndNLP = length(NLPsol.rpm_opt);
    end
    % plot on the motor curves
    figure()
    lgd = [];
    hold on
    [~,effMap] = contourf(rpm_m_eff,T_m_eff,eta_m_fit_eff,[0,70,80,90,95,97,99,100],'ShowText','on');
        map = [255/255,235/255,235/255
              255/255,235/255,235/255
              255/255,235/255,235/255
              255/255,235/255,235/255
              255/255,255/255,235/255
              235/255,255/255,245/255];
        colormap(map)
        lgd = [lgd,"motor efficiency (\%) (FF)"];
    if plotFBMPC
        plot(FBMPCsol.rpm_opt(1:distIndFBMPC),FBMPCsol.Tm_opt(1:distIndFBMPC),'.-','MarkerSize',25)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.rpm_opt(1:distIndABMPC),ABMPCsol.Tm_opt(1:distIndABMPC),'.-','MarkerSize',25)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.rpm_opt(1:distIndBLMPC),BLMPCsol.Tm_opt(1:distIndBLMPC),'.-','MarkerSize',25)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.rpm_opt(1:distIndNLP),NLPsol.Tm_opt(1:distIndNLP),'.-','MarkerSize',25)
        lgd = [lgd,"NLP"];
    end
    plot(rpm_plot,T_m_omega,'k',rpm_plot,-T_m_omega,'k')
        lgd = [lgd,"nonlinear motor limits",""];
    plot([rpm_plot(1),rpm_plot(end)],[ V.T_m_max-V.T_m_max^2/(4*V.P_m_max)*rpm_plot(1)/(30/pi),  V.T_m_max-V.T_m_max^2/(4*V.P_m_max)*rpm_plot(end)/(30/pi)],'k:',...
         [rpm_plot(1),rpm_plot(end)],[-V.T_m_max+V.T_m_max^2/(4*V.P_m_max)*rpm_plot(1)/(30/pi), -V.T_m_max+V.T_m_max^2/(4*V.P_m_max)*rpm_plot(end)/(30/pi)],'k:')
        lgd = [lgd,"linearized motor limits",""];
    legend(lgd,'Interpreter','Latex')
    grid on
    xlim([0,floor(rpm_max)])
    ylim([-260,260])
    ylabel('$T$ (Nm)','Interpreter','Latex')
    xlabel('$\omega$ (rpm)','Interpreter','Latex')
    sdf('LatexFigure');
    set(effMap,'LineWidth',0.1)
    set(effMap,'LineColor',[0.65,0.65,0.65])
    
     % get efficiency map
    [T_m_eff_qdr,rpm_m_eff_qdr,eta_m_fit_eff_qdr] = GetMotorEfficiency(true,OPTsettings);
    if distIndNLP > length(NLPsol.rpm_opt)
        distIndNLP = length(NLPsol.rpm_opt);
    end
    % plot on the motor curves with quadratic efficiency map
    figure()
    lgd = [];
    hold on
    [~,effMap] = contourf(rpm_m_eff_qdr,T_m_eff_qdr,eta_m_fit_eff_qdr,[0,70,80,90,95,97,99,100],'ShowText','on');%,"FaceAlpha",0.25)
        map = [255/255,235/255,235/255
               255/255,235/255,235/255
               255/255,235/255,235/255
               255/255,235/255,235/255
               255/255,255/255,235/255
               235/255,255/255,245/255];
        colormap(map)
        lgd = [lgd,"motor efficiency (\%) (QF)"];
    if plotFBMPC
        plot(FBMPCsol.rpm_opt(1:distIndFBMPC),FBMPCsol.Tm_opt(1:distIndFBMPC),'.-','MarkerSize',25)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.rpm_opt(1:distIndABMPC),ABMPCsol.Tm_opt(1:distIndABMPC),'.-','MarkerSize',25)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.rpm_opt(1:distIndBLMPC),BLMPCsol.Tm_opt(1:distIndBLMPC),'.-','MarkerSize',25)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.rpm_opt(1:distIndNLP),NLPsol.Tm_opt(1:distIndNLP),'.-','MarkerSize',25)
        lgd = [lgd,"NLP"];
    end
    plot(rpm_plot,T_m_omega,'k',rpm_plot,-T_m_omega,'k')
        lgd = [lgd,"nonlinear motor limits",""];
    plot([rpm_plot(1),rpm_plot(end)],[ V.T_m_max-V.T_m_max^2/(4*V.P_m_max)*rpm_plot(1)/(30/pi),  V.T_m_max-V.T_m_max^2/(4*V.P_m_max)*rpm_plot(end)/(30/pi)],'k:',...
         [rpm_plot(1),rpm_plot(end)],[-V.T_m_max+V.T_m_max^2/(4*V.P_m_max)*rpm_plot(1)/(30/pi), -V.T_m_max+V.T_m_max^2/(4*V.P_m_max)*rpm_plot(end)/(30/pi)],'k:')
        lgd = [lgd,"linearized motor limits",""];
    legend(lgd,'Interpreter','Latex')
    grid on
    xlim([0,floor(rpm_max)])
    ylim([-260,260])
    ylabel('$T$ (Nm)','Interpreter','Latex')
    xlabel('$\omega$ (rpm)','Interpreter','Latex')
    sdf('LatexFigure');
    set(effMap,'LineWidth',0.1)
    set(effMap,'LineColor',[0.65,0.65,0.65])
    
    % Acceleration constraint investigation
    figure()
    lgd = [];
    hold on
    if plotFBMPC
        plot(FBMPCsol.v_opt,FBMPCsol.a_opt)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.v_opt,ABMPCsol.a_opt)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.v_opt,BLMPCsol.a_opt)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.v_opt(1:end-1),NLPsol.a_opt)
        lgd = [lgd,"NLP"];
    end
    plot([0,5,20,50,50,20,5,0],[5,5,3.5,3.5,-2,-2,-4,-4],':k')
    lgd = [lgd,"ISO limits"];
    grid on
    legend(lgd,'Interpreter','Latex')
    ylabel('$a$ (m/s$^2$)','Interpreter','Latex')
    xlabel('$v$ (m/s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % Jerk constraint investigation
    figure()
    lgd = [];
    hold on
    if plotFBMPC
        plot(FBMPCsol.v_opt(1:end-1),FBMPCsol.j_opt)
        lgd = [lgd,"FBMPC"];
    end
    if plotABMPC
        plot(ABMPCsol.v_opt(1:end-1),ABMPCsol.j_opt)
        lgd = [lgd,"ABMPC"];
    end
    if plotBLMPC
        plot(BLMPCsol.v_opt(1:end-1),BLMPCsol.j_opt)
        lgd = [lgd,"BLMPC"];
    end
    if plotNLP
        plot(NLPsol.v_opt,NLPsol.j_opt)
        lgd = [lgd,"NLP"];
    end
    plot([0,5,20,50,50,20,5,0],[5,5,3.5,3.5,-3.5,-3.5,-5,-5],':k')
    lgd = [lgd,"ISO limits"];
    grid on
    legend(lgd,'Interpreter','Latex')
    ylabel('$j$ (m/s$^3$)','Interpreter','Latex')
    xlabel('$v$ (m/s)','Interpreter','Latex')
    sdf('LatexFigure');
    
    % running costs
    figure()
    subplot(2,3,1)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_a)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_a)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_a)
            lgd = [lgd,"NLP"];
        end
        grid on
        legend(lgd,'Interpreter','Latex')
        ylabel('acceleration cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    subplot(2,3,2)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_j)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_j)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_j)
            lgd = [lgd,"NLP"];
        end
        grid on
        ylabel('jerk cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    subplot(2,3,3)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_xi_v)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_xi_v)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_xi_v)
            lgd = [lgd,"NLP"];
        end
        grid on
        ylabel('velocity incentive cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    subplot(2,3,4)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_xi_h)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_xi_h)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_xi_h)
            lgd = [lgd,"NLP"];
        end
        grid on
        ylabel('headway slack cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    subplot(2,3,5)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_xi_s)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_xi_s)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_xi_s)
            lgd = [lgd,"NLP"];
        end
        grid on
        ylabel('safety-critical feasibility cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    subplot(2,3,6)
        hold on
        lgd = [];
        if plotFBMPC
            plot(t_plot(1:end-1),FBMPCsol.cost_xi_f)
            lgd = [lgd,"FBMPC"];
        end
        if plotABMPC
            plot(t_plot(1:end-1),ABMPCsol.cost_xi_f)
            lgd = [lgd,"ABMPC"];
        end
        if plotNLP
            plot(t_plot(1:end-1),NLPsol.cost_xi_f)
            lgd = [lgd,"NLP"];
        end
        grid on
        ylabel('general feasibility cost','Interpreter','Latex')
        xlabel('$t$ (s)','Interpreter','Latex')
    sdf('LatexFigure');

end
