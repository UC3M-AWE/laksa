function Jnum=Jacobian(funcion,t,X,PD,PND,cv)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga                        %
% Language  : Matlab                                                         %
% Synopsis  : Compute Jacobian                                               %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %%
% Inputs:  fun   -> function(t,x)                                   %%
%          t     -> time                                            %%
%          X     -> State vector                                    %%
%          PD   -> Dimensionless parameters                        %%
%                                                                   %%
% Outputs: Jnum  -> Jacobian of the function                        %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if PD.Aero.UnPaM == 1 || PD.Aero.UnPaM == 2

    AeroFF     = PD.Aero.FF;
    PD.Aero.FF = 2; % Steady computations

end

switch nargin
    case 5
        for i=1:1:length(X)
            
            XM    = X;
            XM(i) = XM(i)+PD.Num.dh/2;
            fM    = feval(funcion,t,XM,PD,PND);
            
            Xm    = X;
            Xm(i) = Xm(i)-PD.Num.dh/2;
            fm    = feval(funcion,t,Xm,PD,PND);
               
            Jnum(:,i)=(fM-fm)/PD.Num.dh;
        end
        
    case 6
        for i=1:1:length(X)
            
            XM    = X;
            XM(i) = XM(i)+PD.Num.dh/2;
            fM    = feval(funcion,t,XM,cv,PD,PND);
            
            Xm    = X;
            Xm(i) = Xm(i)-PD.Num.dh/2;
            fm    = feval(funcion,t,Xm,cv,PD,PND);
               
            Jnum(:,i)=(fM-fm)/PD.Num.dh;
        end
    otherwise
        error("Jacobian: Incompatible input")
end

if PD.Aero.UnPaM == 1 || PD.Aero.UnPaM == 2

    PD.Aero.FF = AeroFF;

end