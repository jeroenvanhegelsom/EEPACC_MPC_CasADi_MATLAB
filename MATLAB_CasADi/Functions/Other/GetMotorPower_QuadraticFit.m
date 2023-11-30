function [P_bat] = GetMotorPower_QuadraticFit(F_m,rpm,b)
% GetMotorPower_QuadraticFit Calculates the estimated power
%   consumption accoording to a quadratic fit on dynamometer data
%
%   Inputs:
%       F_m : motor force exerted on the road (single value or array)
%       rpm : motor rpm                       (single value or array)
%       b   : coefficients of the fifth order fit
%
%   Output:
%       P_bat : estimated battery power consumption

    P_bat = b(1) + b(2).*F_m + b(3).*rpm + b(4).*F_m.^2 + b(5).*F_m.*rpm + b(6).*rpm.^2;

end
