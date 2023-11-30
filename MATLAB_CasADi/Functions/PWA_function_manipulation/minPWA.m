function [Cdom, Cval] = minPWA(Adom, Aval, Bdom, Bval)
% minPWA Returns a PWA function that is defined by the minimum of two PWA
%   functions at each point in the domain
%
%   Inputs:
%       Adom : array containing the points in the domain that define the PWA function A
%       Aval : array containing the values corresponding the the points in the domain of A
%       Bdom : array containing the points in the domain that define the PWA function B
%       Bval : array containing the values corresponding the the points in the domain of B
%
%   Output:
%       Cdom : array containing the points in the domain that define the PWA function C = min(A,B)
%       Cval : array containing the values corresponding the the points in the domain of C = min(A,B)
    
    % fix start
    if Adom(1) ~= Bdom(1)
        if Adom(1) > Bdom(1)
            Adom = [Bdom(1), Adom];
            Aval = [Aval(1), Aval];
        else % Adom(1) < Bdom(1)
            Bdom = [Adom(1), Bdom];
            Bval = [Bval(1), Bval];
        end
    end
    
    % fix end
    if Adom(end) ~= Bdom(end)
        if Adom(end) > Bdom(end)
            Bdom = [Bdom, Adom(end)];
            Bval = [Bval, Bval(end)];
        else % Adom(end) < Bdom(end)
            Adom = [Adom, Bdom(end)];
            Aval = [Aval, Aval(end)];
        end
    end
    
    % simplify both A and B
    [Adom, Aval] = SimplifyPWA(Adom, Aval);
    [Bdom, Bval] = SimplifyPWA(Bdom, Bval);
    
    % add dummy points
    Adom = [Adom, Adom(end) + 1, Adom(end) + 2];
    Aval = [Aval, Aval(end),     Aval(end)];
    Bdom = [Bdom, Bdom(end) + 1, Bdom(end) + 2];
    Bval = [Bval, Bval(end),     Bval(end)];
    
    % initialize
    Cdom = [];
    Cval = [];
    FullyCheckedA = false;
    FullyCheckedB = false;
    Ad1 = Adom(1);
    Av1 = Aval(1);
    Ad2 = Adom(2);
    Av2 = Aval(2);
    Bd1 = Bdom(1);
    Bv1 = Bval(1);
    Bd2 = Bdom(2);
    Bv2 = Bval(2);
    i_A = 1;
    i_B = 1;
    while true
    
        Aslope = (Av2-Av1)/(Ad2-Ad1);
        Bslope = (Bv2-Bv1)/(Bd2-Bd1);
        
        % check for possible intersection and calculate
        if (Av1 > Bv1 && Av2 < Bv2) || (Av1 < Bv1 && Av2 > Bv2)
    
            s1 = ( Bv1 - Av1 + (Ad1 - Bd1)*Bslope ) / (Aslope - Bslope);
    
            Idom = Ad1 + s1;
            
            % check if intersection is in bounds
            if Idom >= Ad1 && Idom <= Ad2 && Idom >= Bd1 && Idom <= Bd2
                Ival = Av1 + Aslope*s1;
    
                % add intersection to result
                Cdom = [Cdom, Idom];
                Cval = [Cval, Ival];
            end
    
        end
    
        % go to the next overlap
        if Ad2 < Bd2
    
            % check whether to add the starting point of the section
            if Av1 <= InterpPWA(Ad1,Bdom,Bval)
                Cdom = [Cdom, Ad1];
                Cval = [Cval, Av1];
            end
    
            i_A = i_A + 1;
            Ad1 = Adom(i_A);
            Av1 = Aval(i_A);
            Ad2 = Adom(i_A+1);
            Av2 = Aval(i_A+1);
    
            if i_A == length(Adom)-1
                FullyCheckedA = true;
            end
    
        else
    
            % check whether to add the starting point of the section
            if Bv1 <= InterpPWA(Bd1,Adom,Aval)
                Cdom = [Cdom, Bd1];
                Cval = [Cval, Bv1];
            end
    
            i_B = i_B + 1;
            Bd1 = Bdom(i_B);
            Bv1 = Bval(i_B);
            Bd2 = Bdom(i_B+1);
            Bv2 = Bval(i_B+1);
    
            if i_B == length(Bdom)-1
                FullyCheckedB = true;
            end
        end
    
        if FullyCheckedA && FullyCheckedB
            break;
        end
    
    end
    
    % sort C
    [Cdom,sortInds] = sort(Cdom);
    Cval = Cval(sortInds);

end