function s = TimeVelToDist(t,v,s0,M)
% TimeVelToDist Calculates the distance vector given a time and velocity
% vector using M-step forward Euler integration
%
%   Inputs:
%       t : vector with time values
%       v : vector with velocity values
%       s0: initial distance
%       M : number of integration steps per time step
%
%   Output:
%       s : vector with acquired distance values


    % initialize
    n = length(t);
    s = zeros(n,1);
    s(1) = s0;

    % loop over steps
    for i = 1:n-1  
        DT = (t(i+1) - t(i))/M;
        s_ = s(i);
        v_ = v(i);

        % integration in M steps
        for j = 1:M
            s_ = s_ + DT*v_;
        end

        s(i+1) = s_;
    end
end

