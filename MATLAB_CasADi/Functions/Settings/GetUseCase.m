function [OPTsettings] = GetUseCase(OPTsettings)
% GetUseCase Loads the use case characteristics corresponding to the use
%   case number
%
%   Inputs:
%       OPTsettings : struct with optimization settings
%
%   Outputs:
%       OPTsettings : struct with optimization settings containing the loaded use case settings
    
    % select the use case corresponding to the use case number in the settings
    switch OPTsettings.useCaseNum
        case 1 % accelerating from standstill to 80 km/h
            OPTsettings.speedLimZones   = [80, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 300;
            OPTsettings.t_sim = 30;

        case 2 % braking to standstill from 80 km/h
            OPTsettings.v_init          = 80/3.6; % (m/s)
            OPTsettings.speedLimZones   = [80, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [300];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 300;
            OPTsettings.t_sim = 40;

        case 3 % several changes in the speed limit
            OPTsettings.v_init          = 80/3.6; % (m/s)
            OPTsettings.speedLimZones   = [ 80,    0; 
                                           120, 100;
                                            80, 500;
                                            30, 700;
                                            80, 900];
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 1200;
            OPTsettings.t_sim = 100;

        case 4 % approaching a stop
            OPTsettings.v_init          = 0/3.6; % (m/s)
            OPTsettings.speedLimZones   = [80, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [500];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 900;
            OPTsettings.t_sim = 75;

        case 5 % driving a route with several traffic lights
            OPTsettings.v_init          = 80/3.6; % (m/s)
            OPTsettings.speedLimZones   = [80, 0];
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];        
            OPTsettings.TLLoc           = [250, 9, 17, 12
                                           580, 18, 10, 15;
                                           750, 1, 15, 10];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 850;
            OPTsettings.t_sim = 75;

        case 6 % approaching and driving through a curve
            OPTsettings.speedLimZones   = [80,0];
            OPTsettings.slopes          = [];
            OPTsettings.curves          = [-1/20, 100, 130;
                                            1/40, 170, 230;
                                           -1/80, 230, 250];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 400;
            OPTsettings.t_sim = 40;

        case 7 % climbing and descending a hill
            OPTsettings.v_init          = 58/3.6; % (m/s)
            OPTsettings.speedLimZones   = [60, 0]; 
            OPTsettings.slopes          = [0, 1, 300;
                                          (1:8)', (300:10:370)', (310:10:380)';];
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [600];
            OPTsettings.TLLoc           = [];
            % if TV is requested, it must be generated
            OPTsettings.generateTVMPC = true;
            OPTsettings.cutOffDist = 700;
            OPTsettings.t_sim = 70;

        case 8 % driving behind an aggressive target vehicle
            OPTsettings.t_sim = 110;
            OPTsettings.speedLimZones   = [120, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % change TV settings
            OPTsettings.IncludeTV       = true;
            OPTsettings.generateTVMPC   = false;
            % get TV data
            dataTable = readtable('ArgonneData\61505019 Test Data.txt');
            dataTable = table2array(dataTable);
            t_start_read = 4720;
            i_tv_use = dataTable(:,1) >= t_start_read & dataTable(:,1) < t_start_read + OPTsettings.t_sim;
            t_tv = dataTable(i_tv_use,1);
            v_tv = dataTable(i_tv_use,4).*0.44704; % convert to m/s
            s_tv = TimeVelToDist(t_tv,v_tv,OPTsettings.s_init + 20, 5);
            % resample
            resampleFactor = round(OPTsettings.Tvec(1)/diff(t_tv(1:2)));
            OPTsettings.s_tv = [s_tv(1:resampleFactor:end);s_tv(end)];
            OPTsettings.v_tv = [v_tv(1:resampleFactor:end);v_tv(end)];
            OPTsettings.cutOffDist = 800;

        case 9 % driving behind a less aggressive target vehicle
            OPTsettings.t_sim = 270;
            OPTsettings.v_init          = 120/3.6; % (m/s)
            OPTsettings.speedLimZones   = [150, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % change TV settings
            OPTsettings.IncludeTV       = true;
            OPTsettings.generateTVMPC   = false;
            % get TV data
            dataTable = readtable('FINAL_project_files\ArgonneData\61505019 Test Data.txt');
            dataTable = table2array(dataTable);
            t_start_read = 4400; % 
            i_tv_use = dataTable(:,1) >= t_start_read & dataTable(:,1) < t_start_read + OPTsettings.t_sim;
            t_tv = dataTable(i_tv_use,1);
            v_tv = dataTable(i_tv_use,4).*0.44704; % convert to m/s
            s_tv = TimeVelToDist(t_tv,v_tv,OPTsettings.s_init + 50, 5);
            % resample
            resampleFactor = round(OPTsettings.Tvec(1)/diff(t_tv(1:2)));
            OPTsettings.s_tv = [s_tv(1:resampleFactor:end);s_tv(end)];
            OPTsettings.v_tv = [v_tv(1:resampleFactor:end);v_tv(end)];
            OPTsettings.cutOffDist = 8e3;

        case 10 % target vehicle cutting into the lane
            OPTsettings.t_sim = 60;
            OPTsettings.speedLimZones   = [120, 0]; 
            OPTsettings.slopes          = [];           
            OPTsettings.curves          = [];
            OPTsettings.stopLoc         = [];
            OPTsettings.TLLoc           = [];
            % TV settings
            OPTsettings.IncludeTV       = false;
            OPTsettings.generateTVMPC   = false;
            OPTsettings.TV_cuttingDist  = 30; % m
            OPTsettings.TV_cuttingVel   = 100/3.6; % m/s
            OPTsettings.v_init          = 120/3.6;   % initial ego vehicle velocity                                                 (m/s)
            k_tot                       = OPTsettings.t_sim/OPTsettings.Tvec(1) + 1;
            OPTsettings.s_tv            = OPTsettings.TV_cuttingDist + OPTsettings.TV_cuttingVel*OPTsettings.Tvec(1)*(0:k_tot-1)';
            OPTsettings.v_tv            = OPTsettings.TV_cuttingVel*ones(1,k_tot);
            OPTsettings.cutOffDist = 1500;

        case 11 % long route scenario
            OPTsettings.v_init          = 0/3.6; % (m/s)
            OPTsettings.speedLimZones   = [150, 0]; 
            OPTsettings.slopes          = [];
            OPTsettings.speedLimZones   = [ 30,    0; 
                                            50,  600;
                                            80, 1000; 
                                           120, 4450;
                                           80, 10700];      
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
            OPTsettings.stopLoc         = [80;
                                           450;
                                           600;
                                           4000; % traffic light
                                           10700]; % traffic light
            OPTsettings.TLLoc           = [];
            OPTsettings.cutOffDist = 11.5e3;
            OPTsettings.t_sim = 700;
            OPTsettings.generateTVMPC = false;

        case 12 % scenario for the video
            OPTsettings.t_sim = 100;
            OPTsettings.v_init          = 0/3.6; % (m/s)               
            % speed limits:               [max speed (km/h), initial distance (m)]
            OPTsettings.speedLimZones   = [100,   0; 
                                            80, 300;
                                            50, 510];
            
            % const. road slope           [slope (deg), start dist (m), end dist (m)]
            OPTsettings.slopes          = [0, 200, 300];
            
            % curves:                     [curvature (1/m, pos -> ccw), start dist (m), end dist (m)]             
            OPTsettings.curves          = [-1/30, 220, 300;
                                            1/20, 400, 430];
            
            % stops:                      [distance on route from start (m)]
            OPTsettings.stopLoc         = [350; 500];
            
            % traffic lights:             [distance on route from start (m), phase (s), red light time (s), green light time (s)]
            OPTsettings.TLLoc           = [450, 0, 15, 10;
                                           250, 8, 15, 10];
            
            OPTsettings.cutOffDist = 500;
            OPTsettings.generateTVMPC = false;

        otherwise
            error('unknown use case!')
    end
end