function val = InterpPWA(d,doms,vals)
% InterpPWA Interpolates a value in a piecewise affine function
%
%   Inputs:
%       d    : point in the domain to get the interpolated value at
%       doms : array containing the points in the domain that define the PWA function
%       vals : array containing the values corresponding the the points in the domain
%
%   Output:
%       val : resulting interpolated value of the PWA at d

    % check if d is between the bounds of the domain, otherwise set it to
    %   the value of the closest domain point
    if d < doms(1)
        val = vals(1);
    elseif d > doms(end)
        val = vals(end);
    else
        % interpolate
        for i = 1:length(doms)-1
            if d >= doms(i) && d <= doms(i+1)
                f_interp = (d-doms(i))/(doms(i+1)-doms(i));
                val = vals(i) + f_interp*(vals(i+1)-vals(i));
                break;
            end
        end
    end

end