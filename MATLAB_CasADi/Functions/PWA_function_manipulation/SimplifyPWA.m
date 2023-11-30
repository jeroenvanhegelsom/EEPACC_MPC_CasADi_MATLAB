function [domsNew,valsNew] = SimplifyPWA(doms,vals)
% SimplifyPWA Simplifies a piecewise affine function by deleting points
%   that do not add information to its definition
%
%   Inputs:
%       doms : array containing the points in the domain that define the PWA function
%       vals : array containing the values corresponding the the points in the domain
%
%   Output:
%       domsNew : simplified domain array
%       valsNew : simplified value array

    % Delete points where the piece before and after it has the same slope
    doms_(1) = doms(1);
    vals_(1) = vals(1);
    j = 2;
    for i = 2:length(doms)-1
        prevSlope = (vals(i)  -vals(i-1)) / (doms(i)  -doms(i-1));
        currSlope = (vals(i+1)-vals(i))   / (doms(i+1)-doms(i));
        % if the slopes before and after the point are equal, no information is added
        if prevSlope ~= currSlope
            doms_(j) = doms(i);
            vals_(j) = vals(i);
            j = j + 1;
        end
    end
    doms = [doms_, doms(end)];
    vals = [vals_, vals(end)];

    % Delete duplicates
    j = 1;
    for i = 1:length(doms)-1
        if doms(i) == doms(i+1)
            if vals(i) == vals(i+1)
                % don't include
            else
                % shift dom(i) 
                domsNew(j) = doms(i) - .1;
                valsNew(j) = vals(i);
                j = j+1;
            end
        else
            domsNew(j) = doms(j);
            valsNew(j) = vals(j);
            j = j+1;
        end
    end
    domsNew = [domsNew, doms(end)];
    valsNew = [valsNew, vals(end)];

end