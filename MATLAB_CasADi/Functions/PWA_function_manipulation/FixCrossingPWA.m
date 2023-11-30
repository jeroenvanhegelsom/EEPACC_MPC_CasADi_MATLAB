function [doms, vals] = FixCrossingPWA(doms, vals)
% FixCrossingPWA Fixes a PWA that crosses itself by deleting the `loop'
%
%   Inputs:
%       d    : point in the domain to get the interpolated value at
%       doms : array containing the points in the domain that define the PWA function
%       vals : array containing the values corresponding the the points in the domain
%
%   Output:
%       doms : fixed array containing the points in the domain that define the PWA function
%       vals : fixed array containing the values corresponding the the points in the domain
%

    doms_ = doms;

    % find crossing indices
    [~,crossInds] = find(diff(doms) <= 0);

    for i = 1:length(crossInds)

        % find crossing sections A and B
        curCross = crossInds(i);
        Ad1 = doms(curCross - 1);
        Av1 = vals(curCross - 1);
        Ad2 = doms(curCross);
        Av2 = vals(curCross);
        Bd1 = doms(curCross + 1);
        Bv1 = vals(curCross + 1);
        Bd2 = doms(curCross + 2);
        Bv2 = vals(curCross + 2);

        % find crossing value of sections A and B
        Aslope = (Av2-Av1)/(Ad2-Ad1);
        Bslope = (Bv2-Bv1)/(Bd2-Bd1);
        s1 = ( Bv1 - Av1 + (Ad1 - Bd1)*Bslope ) / (Aslope - Bslope);
        Ival = Av1 + Aslope*s1;

        % fix the crossing
        doms(curCross)     = doms_(curCross + 1);
        vals(curCross)     = Ival;
        doms(curCross + 1) = doms_(curCross);
        vals(curCross + 1) = Ival;

    end
    
end