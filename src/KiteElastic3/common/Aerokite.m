function [f, m, alfa, beta] = Aerokite(VA,OMEGA,PD,PND,delta_a,delta_r,delta_e)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Compute aerodynamic force and torque upon the kite             %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %%
% Inputs:  VA    -> components of the airspeed vector              %%
%                   in the body frame                              %%
%          OMEGA -> kite angular velocity components in body frame %% 
%          PD    -> physical parameters of the system              %%
%          PND   -> dimensionless parameters of the system         %%
%          delta_a  -> aileron deflection (rad)                    %%
%          delta_r  -> rudder deflection (rad)                     %%
%          delta_e  -> elevator deflection (rad)                   %%
%                                                                  %%
% Outputs: f     -> components of the aerodynamic force of the     %%
%                   kite in the body frame                         %%
%          m     -> components of the aerodynamic torque about the %%
%                   center of mass of the kite  in the body frame  %%
%          alfa  -> angle of attack   (rad)                        %% 
%          beta  -> sideslip angle    (rad)                        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mu      = PND.Kite.mu; 
eps_c   = PND.Kite.c;      % eps_c
eps_b   = PND.Kite.b;      % eps_b

%%%%% Conversion to dimensionless - KY %%%%%
va = VA/sqrt(PD.Env.g*PD.Tether.L0ref);
T_ast = sqrt(PD.Tether.L0ref/PD.Env.g);
omega = OMEGA * T_ast;   % equiv -> omega / (1 / T_ast))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if PND.Aero.Full == 1
    Full = PND.Aero.Full;
    Vref = []; % DN codegen - Vref has to be defined in both paths.
else
    Full = 0;
    Vref = PND.Aero.vt;    % Vref/sqrt(g*L0)   
end

va2    = va'*va;

if va2 == 0
    alfa   = 0;
    beta   = 0;
    f      = zeros(3,1);
    m      = zeros(3,1); 
else
    %% Attack and sideslip angles
    alfa   = atan2(va(3),va(1));   % original atan(VA(3)/VA(1)) - KY
    beta   = asin(va(2)/sqrt(va2));

    %% Angular velocity in standard flight mechanics normalization
    if Full ==0
        p_tilde = eps_b*omega(1)/(2*Vref);
        q_tilde = eps_c*omega(2)/(Vref);
        r_tilde = eps_b*omega(3)/(2*Vref);
    else
        p_tilde = eps_b*omega(1)/(2*sqrt(va2));
        q_tilde = eps_c*omega(2)/(2*sqrt(va2));
        r_tilde = eps_b*omega(3)/(2*sqrt(va2));
    end
       
    %% Force and moment coefficients in the body frame:

    Aero = PND.Aero; % For codegen, cannot use PND which has function handles in PND.theta_S - KY
    
    [CX, CY, CZ, Cl, Cm, Cn] = Aero_Coefficients(alfa,beta,p_tilde,q_tilde,r_tilde,Aero,delta_a,delta_r,delta_e);
    
%%%%% Convert back to dimensional (consistent with Aerokite_UnPaM) - KY %%%%%
    VA2 = VA' * VA;
    % Aerodynamic force and torque about the center of mass
    f        = zeros(3,1); % initialise - KY
    f(1,1)   =  PD.Kite.mu*VA2*CX; 
    f(2,1)   =  PD.Kite.mu*VA2*CY;
    f(3,1)   =  PD.Kite.mu*VA2*CZ;

    m        = zeros(3,1); % initialise - KY
    m(1,1)   =  PD.Kite.mu*VA2*PD.Kite.b*Cl;
    m(2,1)   =  PD.Kite.mu*VA2*PD.Kite.c*Cm;
    m(3,1)   =  PD.Kite.mu*VA2*PD.Kite.b*Cn;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

end