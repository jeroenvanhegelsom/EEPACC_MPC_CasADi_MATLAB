function [doms,vals] = SaturateSlopePWA(doms,vals,c_desired)
% SaturateSlopePWA  Saturates the slope of the PWA sections to be at most c
%
%   Inputs:
%       doms : array containing the points in the domain that define the PWA function
%       vals : array containing the values corresponding the the points in the domain
%
%   Output:
%       domsNew : domain array with saturated slopes
%       valsNew : value array with saturated slopes

    % fix slopes
    for i = 2:length(doms)
        c = (vals(i) - vals(i-1))/(doms(i) - doms(i-1));
        if c > 0 && c > c_desired
            doms(i) = doms(i-1) + (vals(i) - vals(i-1))/c_desired;
        elseif c < 0 && c < -c_desired
            doms(i-1) = doms(i) + (vals(i) - vals(i-1))/c_desired;
        end
    end

    % Fix remaining crossings
    [doms, vals] = FixCrossingPWA(doms, vals);

    % fix slopes again
    for i = 2:length(doms)
        c = (vals(i) - vals(i-1))/(doms(i) - doms(i-1));
        if c > 0 && c > c_desired
            doms(i) = doms(i-1) + (vals(i) - vals(i-1))/c_desired;
        elseif c < 0 && c < -c_desired
            doms(i-1) = doms(i) + (vals(i) - vals(i-1))/c_desired;
        end
    end

end

