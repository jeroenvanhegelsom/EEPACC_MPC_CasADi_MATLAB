function [OPTsettings] = GenerateUseCase(OPTsettings)
% GenerateUseCase Generates the full use case data set based on the use case characteristics that are set
%
%   Inputs:
%       OPTsettings : struct with optimization settings (includes use case characteristics)
%
%   Outputs:
%       OPTsettings : struct with optimization settings containing the generated use case data

    %% Check values and set to default if empty or undefined
    
    speedLimsEmpty = 0;
    if ~isfield(OPTsettings,'speedLimZones') 
        speedLimsEmpty = 1;
    elseif isempty(OPTsettings.speedLimZones)
        speedLimsEmpty = 1;
    end
    if speedLimsEmpty
        OPTsettings.speedLimZones = [1e5, 0; 1e5, 1];
    end
    curvatureEmpty = 0;
    if ~isfield(OPTsettings,'curves') 
        curvatureEmpty = 1;
    elseif isempty(OPTsettings.curves)
        curvatureEmpty = 1;
    end
    if curvatureEmpty
        OPTsettings.curves = [1/1e5, 1, 2];
    end
    slopesEmpty = 0;
    if ~isfield(OPTsettings,'slopes') 
        slopesEmpty = 1;
    elseif isempty(OPTsettings.slopes)
        slopesEmpty = 1;
    end
    if slopesEmpty
        OPTsettings.slopes = [0, 1, 2];
    end
  
    %% Initialization

    % extract struct fields
    sRes            = OPTsettings.sRes;          % resolution over distance in (m)
    speedLimZones   = OPTsettings.speedLimZones; % speed limits (km/h) in format:                  [max speed (km/h), initial distance (m); ...]
    curves          = OPTsettings.curves;        % curves with constant curvature in format:       [curvature (1/m, pos -> ccw), start dist (m), end dist (m); ...]
    slopes          = OPTsettings.slopes;        % slopes with constant slope angle in format:     [slope (deg), start dist (m), end dist (m); ...]

    %% Generate use case

    % === maximum speed ===

        % initialize
        nSpeedLimZones = size(speedLimZones,1);
        speedLimZones = sortrows(speedLimZones,2);
        speedLimZones = speedLimZones.*repmat([1/3.6, 1],nSpeedLimZones,1); % convert to m/s
        speedLimZones = [speedLimZones; speedLimZones(end,1), 1e5];         % add ending point to set final speed limit to continue to `infinity' (1e5 m)
        v_speedLim = zeros(2*nSpeedLimZones+1,1);
        s_speedLim = v_speedLim;
        % Extend first speed limit to start
        s_speedLim(1) = -1;
        v_speedLim(1) = speedLimZones(1,1);
        % Speed limit sections in the middle
        for i = 1:2:2*nSpeedLimZones
            s_speedLim(i+1) = speedLimZones((i+1)/2,2);
            v_speedLim(i+1) = speedLimZones((i+1)/2,1);
            s_speedLim(i+2) = speedLimZones((i+1)/2+1,2)-1;
            v_speedLim(i+2) = speedLimZones((i+1)/2,1);
        end
        % simplify
        [s_speedLim, v_speedLim] = SimplifyPWA(s_speedLim, v_speedLim);

     % === curvature ===

        % initialize: all 0 curvature
        curvature = zeros(1,max(curves(:,3)));
        % set curvature for all corners
        for i = 1:size(curves,1)
            curvature(curves(i,2):curves(i,3)) = abs(curves(i,1));
        end
        % set min. curvature to 1e-6 to prevent numerical issues caused by division by 0
        for i = 1:length(curvature)
            if abs(curvature(i)) <= 1e-6
                curvature(i) = 1e-6; 
            end
        end
        % corresponding distances
        s_curv = 1:sRes:length(curvature);
        % simplify
        [s_curv, curvature] = SimplifyPWA(s_curv, curvature);
        % set 0 curvature at the end
        s_curv    = [s_curv,    s_curv(end)+1, s_curv(end)+2];
        curvature = [curvature,          1e-6,          1e-6];
        
    % === road slope ===

        % initialize: all 0% slope
        slope = zeros(1,max(slopes(:,3)));
        % set slope angle for all slopes
        for i = 1:size(slopes,1)
            slope(slopes(i,2):slopes(i,3)) = slopes(i,1);
        end
        % convert to radians
        slope = atan(slope/100);
        % corresponding distances
        s_slope = 1:sRes:length(slope);
        % simplify
        [s_slope, slope] = SimplifyPWA(s_slope, slope);

    %% Pack struct fields

    OPTsettings.s_speedLim  = s_speedLim;
    OPTsettings.v_speedLim  = v_speedLim;
    OPTsettings.s_curv      = s_curv;
    OPTsettings.curvature   = curvature;
    OPTsettings.s_slope     = s_slope;
    OPTsettings.slope       = slope;

end