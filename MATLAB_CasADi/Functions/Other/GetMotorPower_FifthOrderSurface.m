function [P_bat] = GetMotorPower_FifthOrderSurface(F_m,rpm,b)
% GetMotorPower_FifthOrderSurface Calculates the estimated power
%   consumption accoording to a fifth order fit on dynamometer data
%
%   Inputs:
%       F_m : motor force exerted on the road (single value or array)
%       rpm : motor rpm                       (single value or array)
%       b   : coefficients of the fifth order fit
%
%   Output:
%       P_bat : estimated battery power consumption

    x = F_m;
    y = rpm;

    P_bat = b(1) + b(2).*x + b(3).*y + b(4).*x.^2 + b(5).*x.*y + b(6).*y.^2 + ...
            b(7).*x.^3 + b(8).*x.^2.*y + b(9).*x.*y.^2 + b(10).*y.^3 + b(11).*x.^4 + ...
            b(12).*x.^3.*y + b(13).*x.^2.*y.^2 + b(14).*x.*y.^3 + b(15).*y.^4 + ...
            b(16).*x.^5 + b(17).*x.^4.*y + b(18).*x.^3.*y.^2 + b(19).*x.^2.*y.^3 + ...
            b(20).*x.*y.^4 + b(21).*y.^5;
end

