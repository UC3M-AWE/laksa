function [X_Amp,Error,Flag] = Equilibrium_New_KA(u0,PD,PND)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga                        %
% Language  : Matlab                                                         %
% Synopsis  : Compute equilibrium state                                      %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Inputs:                                                                  %
%           X0  -> Intial guess (set to zero if an initial guess is unknown) %
%           PD -> Physical Parameters                                        %
%           PND -> Dimensionless Parameters                                  %
%   Outputs:                                                                 %
%           X_Amp -> State vector at the equilibrium                         % 
%           Error -> Equilibrium Error                                       %
%           Flag  -> 1 (successful) or 0 (calculation failed)                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save original control and wind types & change values to compute
% equilibrium:
Control_Type = PD.Ctr.Type;
Wind_Type    = PD.Env.Type;

PD.Env.Type = 2;
PD.Ctr.Type = 0;
cv0 = PD.Tether.L0;
nTethers = length(PD.Tether.L0);

PD.Bridle.Eq = 1; % FLAG COMPRESION

% Forcing steady UnPaM if present and selected:
if PD.Aero.UnPaM == 1 || PD.Aero.UnPaM == 2
    AeroFF     = PD.Aero.FF;
    PD.Aero.FF = 2; % Steady computations
end

if u0 == 0 % Generate an initial guess for the Newton Method
    % Compute ICs:
    the_S0   = 72*pi/180;         % Inclination angle (about -j1) of rg once rotated phi_S0 around -kE
    vg0      = [0; 0; 0];         % Kite velocity in body frame
    Euler0   = [0; 24.45; 0]*pi/180; % Euler angles [psi, theta, phi]
    omegaBE0 = [0; 0; 0];         % Angular rate body-Earth in body frame

    u0 = Fun_Gen_ICs(the_S0, vg0, Euler0, omegaBE0, PD);

    Euler0 = quat2eul(u0(7:10)', 'ZYX')';

    fprintf('Initial guess - Euler theta: %3.2f / Lambda: %3.2f\n',Euler0(2)*180/pi,atan(u0(3)/u0(1))*180/pi);

    
    % Picking up only reduced set of state variables:
    X0 = [u0(1); u0(3); Euler0(2); u0(14:13+3*nTethers*PD.Tether.N)]; % [rg_x, rg_z, theta, {rm}] (3+3*2*N)

else % Take input IG

    % Getting Euler angles from quaternions:
    Euler0 = quat2eul(u0(7:10)');

    % Picking up only reduced set of state variables:
    X0 = [u0(1); u0(3); Euler0(2); u0(14:13+3*nTethers*PD.Tether.N)]; % [rg_x, rg_z, theta, {rm}] (3+3*2*N)
    
end

% Call Newton-Raphson method:
options = optimoptions(@fsolve,'MaxFunctionEvaluations',200000,'MaxIterations',2000);
%[X, Error, Flag] = my_fzero(@Equilibrium,X0,PD);
[X, Error, Flag] = fsolve(@(X) Equilibrium(X), X0, options);
if Flag == 0
    error('Could not find equilibrium!')
end

% Recover the full State-Vector:
rg      = [X(1); 0; X(2)];             % Position vector (SE components)
vg      = zeros(3,1);                  % Velocity vector (SB components)
q       = eul2quat([0 X(3) 0],'ZYX')'; % Quaternions 
omegaBE = zeros(3,1);                  % Angular velocity (SB components)
X_Amp   = [rg; vg; q; omegaBE];   

% Add masses positions:
X_Amp = [X_Amp; X(4:3+3*nTethers*PD.Tether.N)];

% Add masses velocities:
X_Amp = [X_Amp; zeros(3*nTethers*PD.Tether.N,1)];

% Set original control and wind types
PD.Ctr.Type = Control_Type;
PD.Env.Type = Wind_Type;

PD.Bridle.Eq = 0;

% Set original aero settings:
if PD.Aero.UnPaM == 1 || PD.Aero.UnPaM == 2

    PD.Aero.FF = AeroFF;

end


%% Internal functions:

% Flow of dynamical system:
function DF = Equilibrium(X_Eq)

    % Kites variables:
    r       = [X_Eq(1); 0; X_Eq(2)];          % Position vector (SE components)
    v       = zeros(3,1);                     % Velocity vector (SB components)
    q       = eul2quat([0 X_Eq(3) 0],'ZYX')'; % Quaternions 
    omegaBE = zeros(3,1);                     % Angular velocity (SB components)
    X_Amp   = [r; v; q; omegaBE];
          
    % Discrete masses positions:
    X_Amp = [X_Amp; X_Eq(4:3+3*nTethers*PD.Tether.N)];
    
    % Discrete masses velocities:
    X_Amp = [X_Amp; zeros(3*nTethers*PD.Tether.N,1)];
    
    % Call de full RHS:
    DF0   = Fun_ODE_KE3(0,X_Amp,cv0,PD,PND);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Compute the reduced RHS %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    DF(1,1)                   = DF0(4,1);  % dvx/dt = 0
    DF(2,1)                   = DF0(6,1);  % dvz/dt = 0
    DF(3,1)                   = DF0(12,1); % domegaBE_y/dt  = 0
    DF(4:3+3*nTethers*PD.Tether.N,1) = DF0(14+3*nTethers*PD.Tether.N:13+2*3*nTethers*PD.Tether.N); % dvm/dt = 0

    clear X_Amp
    
end

end