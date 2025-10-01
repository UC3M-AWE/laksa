function PND = Fun_PND_KA(PD)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Compute Dimensionless parameters                               %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs: structure PD with the physical parameters                %%
%                                                                  %%
% Outputs: structure PND with the dimensionless parameters         %%
%                                                                  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Kite Geometry and Inertia
PND.Kite.mu      =  PD.Env.ro*PD.Kite.A*PD.Tether.L0ref/(2*PD.Kite.m );    % mu
PND.Kite.c       =  PD.Kite.c/PD.Tether.L0ref;                             % eps_c
PND.Kite.b       =  PD.Kite.b/PD.Tether.L0ref;                             % eps_b
PND.Kite.h       =  PD.Kite.h/PD.Tether.L0ref;                             % eps_h (only plotting purposes)
PND.Kite.hg      =  PD.Kite.hg/PD.Tether.L0ref;                            % eps_hg (only plotting purposes)

iota_Ix          = PD.Kite.Ix/(PD.Kite.m*PD.Tether.L0ref^2);               % Ix/m*L0^2   
iota_Iy          = PD.Kite.Iy/(PD.Kite.m*PD.Tether.L0ref^2);               % Iy/m*L0^2   
iota_Iz          = PD.Kite.Iz/(PD.Kite.m*PD.Tether.L0ref^2);               % Iz/m*L0^2    
iota_Ixz         = PD.Kite.Ixz/(PD.Kite.m*PD.Tether.L0ref^2);              % Ixz/m*L0^2 

PND.Kite.iota    = [iota_Ix 0 iota_Ixz; 0 iota_Iy 0;iota_Ixz 0  iota_Iz];

%% Force Aerodynamic coefficients

PND.Aero.Full =  PD.Aero.Full;

if PD.Aero.Full ==  1
    PND.Aero.CX = PD.Aero.CX;
    PND.Aero.CY = PD.Aero.CY;
    PND.Aero.CZ = PD.Aero.CZ;
    
    PND.Aero.Cm = PD.Aero.Cm;
    PND.Aero.Cl = PD.Aero.Cl;
    PND.Aero.Cn = PD.Aero.Cn;
else    
    
    PND.Aero.CX = [PD.Aero.Cxalpha2  PD.Aero.Cxalfa PD.Aero.Cx0...    % CX0 
                   0         0              0 ...         % CX_beta
                   0         0              0 ...         % CX_p
                   0         0              0 ...         % CX_q
                   0         0              0 ...         % CX_r 
                   0         0              0 ...         % CX_delta_aileron
                   0         0              0 ...         % CX_delta_elevator
                   0         0              0];           % CX_delta_rudder     

    PND.Aero.CY =  [0         0              0 ...         % CY0 
                   0         0        PD.Aero.Cybeta ...  % CY_beta
                   0         0              0  ...        % CY_p
                   0         0              0  ...        % CY_q
                   0         0              0  ...        % CY_r 
                   0         0              0  ...        % CY_delta_aileron
                   0         0       PD.Aero.Cydelta_r... % CY_delta_elevator
                   0         0              0  ];         % CY_delta_rudder     
             
    PND.Aero.CZ = [ 0    PD.Aero.Czalfa  PD.Aero.Cz0 ...   % CZ0 
                   0         0              0 ...         % CZ_beta
                   0         0              0 ...         % CZ_p
                   0         0              0 ...         % CZ_q
                   0         0              0 ...         % CZ_r 
                   0         0              0 ...         % CZ_delta_aileron
                   0         0              0 ...         % CZ_delta_elevator
                   0         0              0];           % CZ_delta_rudder     
              

    PND.Aero.Cm = [ 0      PD.Aero.Cmalfa  PD.Aero.Cm0 ... % Cm0 
                   0         0              0 ...         % Cm_beta
                   0         0              0 ...         % Cm_p
                   0         0            PD.Aero.Cmq...  % Cm_q
                   0         0              0 ...         % Cm_r 
                   0         0              0 ...         % Cm_delta_aileron
                   0         0       PD.Aero.Cmdelta_e ...% Cm_delta_elevator
                   0         0              0];           % Cm_delta_rudder     

     PND.Aero.Cl= [ 0         0              0 ...         % Cl0 
                   0         0         PD.Aero.Clbeta ... % Cl_beta
                   0         0            PD.Aero.Clp ... % Cl_p
                   0         0              0  ...        % Cl_q
                   0         0              0 ...         % Cl_r 
                   0         0      PD.Aero.Cldelta_a ... % Cl_delta_aileron
                   0         0              0 ...         % Cl_delta_elevator
                   0         0      PD.Aero.Cldelta_r];   % Cl_delta_rudder   
               
     PND.Aero.Cn= [ 0         0             0 ...         % Cn0 
                   0         0         PD.Aero.Cnbeta ... % Cn_beta
                   0         0              0         ... % Cn_p
                   0         0              0  ...        % Cn_q
                   0         0           PD.Aero.Cnr...   % Cn_r 
                   0         0              0 ... % Cn_delta_aileron
                   0         0              0 ...         % Cn_delta_elevator
                   0         0      PD.Aero.Cndelta_r ];  % Cn_delta_rudder         
end 

PND.Aero.vt      = PD.Aero.Vref/sqrt(PD.Env.g*PD.Tether.L0ref);            % V_ref/sqrt(g*L0)     

% Aerodynamic Model Limits (only for postprocess checking purposes)
PND.Aero.alfa_s =  PD.Aero.alfa_s*pi/180;                               % Stall angle(rad)
PND.Aero.beta_m =  PD.Aero.beta_m*pi/180;   

%% Enviromental Properties
PND.Env.Type     = PD.Env.Type;                                         % Wind law
PND.Env.vw       = PD.Env.Vw/sqrt(PD.Env.g*PD.Tether.L0ref);               % V_0 tilde 
PND.Env.alfa     = PD.Env.alfa;                                         % Exponent of the wind speed law
PND.Env.H0       = PD.Env.H0/PD.Tether.L0ref;                              % Height Scale
PND.Env.eps      = PD.Env.eps;                                          % Wind Speed fluctuation level
PND.Env.Omega    = PD.Env.Omega*sqrt(PD.Tether.L0ref/PD.Env.g);            % Normalized fluctuation frequency 

%% Control parameters (unused)
PND.Ctr.Type     = PD.Ctr.Type;                                         % 0 ->  l = sqrt(1-yA^2) and delta = 0
                                                                        % 1 ->  l = sqrt(1-yA^2) and delta = delta1*sin(omega*t)                                 
                                                                        % 2 ->  l = sqrt(1-yA^2) + l1*sin(omega_l*t) and delta = delta1*sin(omega_delta*t) 
PND.Ctr.l1       = PD.Ctr.l1/PD.Tether.L0ref;                              % l1/L0
PND.Ctr.Om_l     = PD.Ctr.Om_l*sqrt(PD.Tether.L0ref/PD.Env.g);             % Normalized omega_l1
PND.Ctr.delta1   = PD.Ctr.delta1*pi/180;                                % delta1 (rad)         
PND.Ctr.Om_delta = PD.Ctr.Om_delta*sqrt(PD.Tether.L0ref/PD.Env.g);         % Normalized omega_delta 
                
%% Numerical Parameters
PND.Num.RelTol   = PD.Num.RelTol;        % Integrator Relative Tolerance
PND.Num.AbsTol   = PD.Num.AbsTol;        % Integrator Absolute Tolerance
PND.Num.NewTol   = PD.Num.NewTol;        % Newton-Raphson Tolerance
PND.Num.MaxIt    = PD.Num.MaxIt;         % Newton-Raphson Maximum number of iterations
PND.Num.DTmax    = PD.Num.DTmax;         % Maximum Time step
PND.Num.dh       = PD.Num.dh;            % Numerical Jacobian step
end