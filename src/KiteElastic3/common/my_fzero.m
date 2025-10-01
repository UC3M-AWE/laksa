function [X Error Flag]=my_fzero(fun,X,PND)

%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : Gonzalo Sanchez-Arriaga, Alejandro Pastor-Rodriguez,           %
% Language  : Matlab                                                         %
% Synopsis  : Newton Method                                                  %
% Copyright:  Universidad Carlos III de Madrid, 2017. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %%
% Inputs:  fun   -> function(t,x)                                   %%
%          X     -> Initial guess                                   %%
%          PND   -> Dimensionless parameters                        %%
%                                                                   %%
% Outputs: X     -> Vector that makes | function(0,X) <Error        %%
%          Error -> Error of the solution                           %%
%          Flag  -> 1-> Success, 0 -> The method did not converge   %%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize the flag
Flag   = 0;
format long
for j=1:1:PND.Num.MaxIt
    
    F0    = feval(fun,0,X);
    Error = sqrt(F0'*F0);
    if Error<PND.Num.NewTol
        Flag = 1;
        break
    else
        %Compute the Jacobian 
        J  = Jacobian(fun,0,X,PND);  
        % Correct the solution
        X = X - J\F0;  
    end
end




end

