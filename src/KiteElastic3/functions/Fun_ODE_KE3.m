function [f] = Fun_ODE_KE3(t, xs, cv, PD, PND)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : RHS of the equations (Newtonian formulation)                   %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %%
% Inputs:  t   -> Dimensional time                                  %%
%          xs  -> Extended state vector                             %%
%                 [{rg} {vg} {q} {omegaBE} {rp} {vp}]               %%
% Outputs: f   -> Right-Hand-Side of dynamical system eqs.          %%
%                                                                   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Calling full output routine and only take the dynamical system flow:

[~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, f] =...
    Fun_ODE_Full_Output_KE3(t, xs, cv, PD, PND);
 
end
