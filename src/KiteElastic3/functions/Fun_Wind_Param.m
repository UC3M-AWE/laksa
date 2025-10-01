function vw  = Fun_Wind_Param(t,rg,type,vw0)

%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Compute Wind velocity                                          %
% Copyright:  Universidad Carlos III de Madrid, 2017. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %%
% Inputs:  t                  -> dimensionless time                %%
%          rg                 -> dimensionless vector position of  %%
%                                the center of mass of the kite    %% 
%                                (components in SE)                %%
%          PND                -> dimensionless parameters          %%
%                                                                  %%
% Outputs: vw                 -> components in SE of the           %%
%          dimensionless  wind velocity vector                     %%
%                                                                  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vw = zeros(3,1); % specify for all paths if switch neq 0 or 2
switch type
    case 0 % Constant wind speed
        vw = [-vw0;0;0];
    case 1 % Wind Speed = Vw*(h/H0)^alfa*(1+eps*sin(Omega*t))
      
%         H  = -rg(3);  % Normalized kite altitude
%         vw = -PND.Env.vw*(H/PND.Env.H0)^PND.Env.alfa*(1+PND.Env.eps*sin(PND.Env.Omega*t))*[1 0 0]';
         % specify for codegen path as vw above disabled due to no PND input - KY
    case 2 %log model
        H  = -rg(3);
        zr = 1e-2;
        u0 = vw0;
        z0 = 5;
        if H<zr
            vw = [0 0 0]';
        else
            vw = (u0*log(H/zr)/log(z0/zr))*[-1 0 0]';
        end
end


end