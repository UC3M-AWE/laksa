function PD = Fun_PD_RFD(p)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Physical Parameters                                            %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs: No inputs                                                %%
%                                                                  %%
% Outputs: structure PD with the physical parameters               %% 
%                                                                  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Kite geometry and physical characteristics  
PD.Kite.c     =  0.5933;           % Kite Chord                          (m)
PD.Kite.b     =  3.66;             % Kite span                           (m)
PD.Kite.m     =  p(4);             % Kite mass                           (kg)
PD.Kite.h     =  0.5;              % Kite height                         (m, only plotting purposes)
PD.Kite.hg    =  0.3;              % Z-Distance between G and kite tips  (m, only plotting purposes)
PD.Kite.A     =  1.8694;           % Kite surface                        (m^2)  

PD.Kite.Ix    =  0.715;            % Ix                                  (kg m^2)
PD.Kite.Iy    =  0.115;            % Iy                                  (kg m^2)
PD.Kite.Iz    =  0.819;            % Iz                                  (kg m^2) 
PD.Kite.Ixz   =  -0.003;           % Ixz                                 (kg m^2)

PD.Kite.IG     = zeros(3,3); % initialise codegen - KY
PD.Kite.IG_inv = zeros(3,3); % initialise codegen - KY
PD.Kite.mu     = zeros(1,1); % initialise codegen - KY




PD.Bridle.Anchors = [   [-0.149+0.066,        0,   0.015];
                        [-0.149+0.066,        0,   0.015];
                        [-0.149+0.066,    1.380,   0.015];
                        [-0.149+0.066,   -1.380,   0.015];
                        [0.486-0.08,      0.45,   0.015];
                        [0.486-0.08,     -0.45,   0.015];
                        [0.166-0.066,     0.930,   0.015];
                        [0.166-0.066,    -0.930,   0.015]];

PD.Bridle.Lengths  = [  1.25 - 0.0312;
                        1.25 - 0.0312;
                        1.20 - 0.03;
                        1.20 - 0.03;
                        0.75 - 0.0188;
                        0.75 - 0.0188;
                        1.10 - 0.0275;
                        1.10 - 0.0275];

PD.Bridle.Connectivity = {[1,3,7];[2,4,8];[5,6]};

PD.Anchor.Points =[ [0,     0.35,       0];
                    [0,     -0.35,      0]
                    [0.05,     0,      0]];

% L0 main line length (m)
PD.Tether.L0ref = 39.28;
PD.Tether.L0 = [p(5);p(6);p(7)];

PD.Bridle.Eq = 0; % 0/1 for no/yes equilibrium computation (to remove compression constraint)

%% Force Aerodynamic coefficients:
% Only base model provided for the moment.
PD.Aero.ModelSwitch = 0;                 % 0 base model / UnPaM / Quasi-Steady
PD.Aero.Full      = 0;                   % Set 1 to use full model  
PD.Aero.UnPaM     = 0;                   % UnPaM aerodynamic flag --> 0/1/2 for analytical/UnPaM/Mixed aero model
% ONLY PD.Aero.UnPaM=0 SWITCH ALLOWED
PD.Aero.UnPaM_Mix = [1 1 1 0 0 0];       % 1/0 for UnPaM/analytical in this order [CX CY CZ Cl Cm Cn] (only valid if UnPaM == 2)
PD.Aero.Post      = 0;                   % 0/1 for compute (real-time run)/postprocess (interpolate already computed values) UnPaM coeffs
% PD.Aero.Post unused
PD.Aero.NamePost  = 'UnPaM_Results_DT.mat'; % Name for saving integration results together with UnPaM computations
PD.Aero.NP        = 12;                  % Number of processors for UnPaM to run

PD.Aero.Flag_TS = 1; % 0/1 for no/yes compute a time step with UnPaM
PD.Aero.FF      = 0; % 0/1/2 for running UnPaM time step with changing wake/fixed wake/steady computation
PD.Aero.DT      = 1e-2; % Time step for UnPaM (this should be adjusted in the integrator)
PD.Aero.t       = zeros(1,1); % unused, but for codegen - KY

% Force Aerodynamic derivatives (UnPaM)
PD.Aero.Cx0      = -1.055;        % Cx0                          (-)  
PD.Aero.Cxalfa   = 2.846;       % Cx_alfa                      (-) 
PD.Aero.Cxalpha2 = 0;        % Cx_alpha^2   
PD.Aero.Cybeta   = -0.5066;      % Cy_beta                      (-)  
PD.Aero.Cz0      = 0.924;        % Cz0                          (-)  
PD.Aero.Czalfa   = -3.729;       % Cz_alfa                      (-)  

% Torque Aerodynamic derivatives (experimental)
PD.Aero.Clbeta = -0.01579;       % Cl_beta                      (-)     
PD.Aero.Clp    = -0.01972;       % Cl_p_tilde                   (-)    
PD.Aero.Cm0    = -0.4147;        % Cm0                          (-)    
PD.Aero.Cmalfa = 1.11;           % Cm_alfa                      (-)    
PD.Aero.Cmq    = -0.5623;        % Cm_q_tilde                   (-)      
PD.Aero.Cnbeta = 0.004493;       % Cn_beta                      (-)    
PD.Aero.Cnr    = -0.00399;       % Cn_r_tilde                   (-)       
     
PD.Aero.Vref   = 10;             % V_ref                        (m/s)

% Control ->  This control is not implemented in the current version of the code
PD.Aero.Cydelta_r = 0;
PD.Aero.Cmdelta_e = 0;
PD.Aero.Cldelta_a = 0;
PD.Aero.Cldelta_r = 0;
PD.Aero.Cndelta_r = 0;  

% Aerodynamic Model Limits (only for postprocess checking purposes)
PD.Aero.alfa_s =  50;            % Stall angle                  (deg)
PD.Aero.beta_m =  20;            % Maximum sideslip angle       (deg)

% Construct aerodynamic coefficient model
% PD.Aero.Coefficients moved to end for codegen - KY
PD.Aero.CX = zeros(1,24); PD.Aero.Cl = zeros(1,24); % initialise codegen - KY
PD.Aero.CY = zeros(1,24); PD.Aero.Cm = zeros(1,24); % initialise codegen - KY
PD.Aero.CZ = zeros(1,24); PD.Aero.Cn = zeros(1,24); % initialise codegen - KY
PD.Aero.vt = zeros(1,1); % initialise codegen - KY

%% Environment parameters
PD.Env.g      =  9.81;  % Earth acceleration                (m/s^2)
PD.Env.ro     =  1.225; % Air density                       (kg/m^3)
PD.Env.Type   =  0;     % 0 -> Wind Velocity is constant
                        % 1 -> Wind Speed = Vw*(h/H0)^alfa*(1+eps*sin(Omega*t))
PD.Env.Vw     = p(1); % 10 default      % Wind Velocity                     (m/s)
PD.Env.alfa   = 0.14;   % Exponent of the wind speed law
PD.Env.H0     = 10;     % Height Scale                      (m)
PD.Env.eps    = 0.1;    % Wind Speed fluctuation level
PD.Env.Omega  = 0.6;    % Wind Speed fluctuation frequency  (rad/s)

PD.Env.vw     = zeros(1,1); % initialise codegen - KY

%% Tether characteristics
PD.Tether.A   = pi*1e-3^2;         % Cross area of the tether (m^2)
PD.Tether.rho = 0.0018;            % Length density (kg/m)
PD.Tether.N   = 10;                % Number of spring-damper elements per tether (line)
PD.Tether.E   = 1.8e11;            % Young's modulus of the tether and bridle springs (Pa)
PD.Tether.C   = 1;                 % Damping of the tether and bridle dampers (N/(m/s))
PD.Tether.CDt = 1.1;               % Tether drag coefficient (-)

%% Control parameters (unused, not implemented in the current version of the code) 
PD.Ctr.Type   = 0;               % 0 ->  l = sqrt[1-(YA/L0)^2] and delta = 0
                                 % 1 ->  l = sqrt[1-(YA/L0)^2] and delta = delta1*sin(omega*t)                            
                                 % 2 ->  l = sqrt[1-(YA/L0)^2] + l1*sin(omega_l*t) and delta = delta1*sin(omega_delta*t)
                                 % 3 ->  Gonzalo's law with flat stages
                                 % 4 ->  l = sqrt[1-(YA/L0)^2] and delta = Experimental law fitted
                                 % 5 ->  l = sqrt[1-(YA/L0)^2] and delta = Closed-loop control    
                                 % 6 ->  l = sqrt[1-(YA/L0)^2] and delta = geometric delta (Fagiano's)
PD.Ctr.l1       = 1;             % l1 (m)
PD.Ctr.Om_l     = 0;             % omega_l (rad/s)
PD.Ctr.delta1   = p(2); % 0.1 default   % delta1 (m)
PD.Ctr.Om_delta = p(3); %0.5 default    % omega_delta (rad/s) 
PD.Ctr.Om_T     = 0;             % delta period  if PD.Ctr.Type   = 3; (s) 
PD.Ctr.t0       = 0;             % Time shift (s) 
PD.Ctr.Rate     = 40;            % Controller Rate (Hz)

% For close-loop control (5):
PD.Ctr.TP          = [0.17, 0.28]; % [(+-)phi, theta] of two Target Points (spherical coordinates)
PD.Ctr.Kc          = 10*1e-4; % -7.5e-5; % 5e-4; % Controller (velocity angle control) prop. gain (1/rad). It should be (m/rad) in dimensional form
PD.Ctr.Lim_delta_c = 20*1e-5;
PD.Ctr.Kdelta      = -103; % Controller (position control) gain (Ddelta/Ddelta_m (rad)). It should be (rad/m) in dimensional form
PD.Ctr.a           = 0.22; % Longitudinal distance between first line of pulleys and carriage (m)
PD.Ctr.b           = 0.22; % Longitudinal distance between carriage and second line of pulleys (m)
PD.Ctr.c           = 0.37; % 1/2 of the total effective linear actuator length (m)
PD.Ctr.delta_m_Lim = 1.5;  % PD.Ctr.c; % Bounds of the linear actuator (m)

% For geometric control (6):
PD.Ctr.Ec        = 1;
PD.Ctr.Es        = 1;
PD.Ctr.theta_ref = 20*pi/180;
PD.Ctr.d         = 0.5;

%% Numerical Parameters
PD.Num.RelTol  = 1e-6;           % Integrator Relative Tolerance
PD.Num.AbsTol  = 1e-6;           % Integrator Absolute Tolerance
PD.Num.NewTol  = 1e-6;           % Newton-Raphson Tolerance
PD.Num.MaxIt   = 1000;           % Newton-Raphson Maximum number of iterations
PD.Num.DTmax   = 0.001;          % Maximum Time step
PD.Num.dh      = 1e-6;           % Numerical Jacobian step

if PD.Aero.Full ==  1
    PD.Aero.CX = PD.Aero.CX;
    PD.Aero.CY = PD.Aero.CY;
    PD.Aero.CZ = PD.Aero.CZ;
    
    PD.Aero.Cm = PD.Aero.Cm;
    PD.Aero.Cl = PD.Aero.Cl;
    PD.Aero.Cn = PD.Aero.Cn;

else    
    PD.Aero.CX = [PD.Aero.Cxalpha2  PD.Aero.Cxalfa PD.Aero.Cx0...       % CX0 
                   0         0              0 ...         % CX_beta
                   0         0              0 ...         % CX_p
                   0         0              0 ...         % CX_q
                   0         0              0 ...         % CX_r 
                   0         0              0 ...         % CX_delta_aileron
                   0         0              0 ...         % CX_delta_elevator
                   0         0              0];           % CX_delta_rudder    

    PD.Aero.CY =  [0         0              0 ...         % CY0 
                   0         0        PD.Aero.Cybeta ...  % CY_beta
                   0         0              0  ...        % CY_p
                   0         0              0  ...        % CY_q
                   0         0              0  ...        % CY_r 
                   0         0              0  ...        % CY_delta_aileron
                   0         0       PD.Aero.Cydelta_r... % CY_delta_elevator
                   0         0              0  ];         % CY_delta_rudder     
             
    PD.Aero.CZ = [ 0    PD.Aero.Czalfa  PD.Aero.Cz0 ...   % CZ0 
                   0         0              0 ...         % CZ_beta
                   0         0              0 ...         % CZ_p
                   0         0              0 ...         % CZ_q
                   0         0              0 ...         % CZ_r 
                   0         0              0 ...         % CZ_delta_aileron
                   0         0              0 ...         % CZ_delta_elevator
                   0         0              0];           % CZ_delta_rudder     
              

    PD.Aero.Cm = [ 0      PD.Aero.Cmalfa  PD.Aero.Cm0 ... % Cm0 
                   0         0              0 ...         % Cm_beta
                   0         0              0 ...         % Cm_p
                   0         0            PD.Aero.Cmq...  % Cm_q
                   0         0              0 ...         % Cm_r 
                   0         0              0 ...         % Cm_delta_aileron
                   0         0       PD.Aero.Cmdelta_e ...% Cm_delta_elevator
                   0         0              0];           % Cm_delta_rudder     

     PD.Aero.Cl= [ 0         0              0 ...         % Cl0 
                   0         0         PD.Aero.Clbeta ... % Cl_beta
                   0         0            PD.Aero.Clp ... % Cl_p
                   0         0              0  ...        % Cl_q
                   0         0              0 ...         % Cl_r 
                   0         0      PD.Aero.Cldelta_a ... % Cl_delta_aileron
                   0         0              0 ...         % Cl_delta_elevator
                   0         0      PD.Aero.Cldelta_r];   % Cl_delta_rudder   
               
     PD.Aero.Cn= [ 0         0             0 ...         % Cn0 
                   0         0         PD.Aero.Cnbeta ... % Cn_beta
                   0         0              0         ... % Cn_p
                   0         0              0  ...        % Cn_q
                   0         0           PD.Aero.Cnr...   % Cn_r 
                   0         0              0 ... % Cn_delta_aileron
                   0         0              0 ...         % Cn_delta_elevator
                   0         0      PD.Aero.Cndelta_r ];  % Cn_delta_rudder         
end

% Defining dimensional params to be used as PND (tricking) in certain functions:
PD.Kite.IG     = [PD.Kite.Ix 0 PD.Kite.Ixz; 0 PD.Kite.Iy 0;PD.Kite.Ixz 0  PD.Kite.Iz];
PD.Kite.IG_inv = inv(PD.Kite.IG); % We inverse it once to save time
PD.Kite.mu     = 1/2*PD.Env.ro*PD.Kite.A; % Parameter that multiplies the force and moment coeffs to get magnitudes with dimensions

PD.Env.vw = PD.Env.Vw;
PD.Aero.vt = PD.Aero.Vref;

end