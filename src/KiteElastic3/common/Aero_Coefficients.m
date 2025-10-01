function [CX CY CZ Cl Cm Cn ] = Aero_Coefficients(alfa,beta,p,q,r,Aero,delta_a,delta_r,delta_e)

%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Compute aerodynamic force and torque upon the kite             %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %%
% Inputs:  alfa, beta -> angle of attck and sideslip angle         %%
%          p,q,r      -> normalized angular velocity components    %% 
%          PND   -> dimensionless parameters of the system         %%
%          delta_a  -> aileron deflection (rad)                    %%
%          delta_r  -> rudder deflection (rad)                     %%
%          delta_e  -> elevator deflection (rad)                   %%
%                                                                  %%
% Outputs: Cx,Cy,Cz -> aerodynamic force coeffcients               %%
%          Cl,Cm,Cn -> aerodynamic torque coefficient              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vec_alfa = [alfa^2 alfa 1]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  CX  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CX0     = Aero.CX(1:3)*Vec_alfa; 
CX_beta = Aero.CX(4:6)*Vec_alfa;
CX_p    = Aero.CX(7:9)*Vec_alfa;
CX_q    = Aero.CX(10:12)*Vec_alfa;
CX_r    = Aero.CX(13:15)*Vec_alfa;
CX_ail  = Aero.CX(16:18)*Vec_alfa;
CX_ele  = Aero.CX(19:21)*Vec_alfa;
CX_rud  = Aero.CX(22:24)*Vec_alfa;

CX      = CX0 + CX_beta*beta + CX_p*p + CX_q*q  + CX_r*r + CX_ail*delta_a + CX_ele*delta_e  + CX_rud*delta_r;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  CY  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CY0     = Aero.CY(1:3)*Vec_alfa; 
CY_beta = Aero.CY(4:6)*Vec_alfa;
CY_p    = Aero.CY(7:9)*Vec_alfa;
CY_q    = Aero.CY(10:12)*Vec_alfa;
CY_r    = Aero.CY(13:15)*Vec_alfa;
CY_ail  = Aero.CY(16:18)*Vec_alfa;
CY_ele  = Aero.CY(19:21)*Vec_alfa;
CY_rud  = Aero.CY(22:24)*Vec_alfa;

CY      = CY0 + CY_beta*beta + CY_p*p + CY_q*q  + CY_r*r + CY_ail*delta_a + CY_ele*delta_e  + CY_rud*delta_r;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  CZ  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CZ0     = Aero.CZ(1:3)*Vec_alfa; 
CZ_beta = Aero.CZ(4:6)*Vec_alfa;
CZ_p    = Aero.CZ(7:9)*Vec_alfa;
CZ_q    = Aero.CZ(10:12)*Vec_alfa;
CZ_r    = Aero.CZ(13:15)*Vec_alfa;
CZ_ail  = Aero.CZ(16:18)*Vec_alfa;
CZ_ele  = Aero.CZ(19:21)*Vec_alfa;
CZ_rud  = Aero.CZ(22:24)*Vec_alfa;

CZ      = CZ0 + CZ_beta*beta + CZ_p*p + CZ_q*q  + CZ_r*r + CZ_ail*delta_a + CZ_ele*delta_e  + CZ_rud*delta_r;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  Cl  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cl0     = Aero.Cl(1:3)*Vec_alfa; 
Cl_beta = Aero.Cl(4:6)*Vec_alfa;
Cl_p    = Aero.Cl(7:9)*Vec_alfa;
Cl_q    = Aero.Cl(10:12)*Vec_alfa;
Cl_r    = Aero.Cl(13:15)*Vec_alfa;
Cl_ail  = Aero.Cl(16:18)*Vec_alfa;
Cl_ele  = Aero.Cl(19:21)*Vec_alfa;
Cl_rud  = Aero.Cl(22:24)*Vec_alfa;

Cl      = Cl0 + Cl_beta*beta + Cl_p*p + Cl_q*q  + Cl_r*r + Cl_ail*delta_a + Cl_ele*delta_e  + Cl_rud*delta_r;   

% %% Limiting moment coefficients to the max alpha and beta allowed:
% 
% if abs(alfa) > PND.Aero.alfa_s
%    
%     alfa = PND.Aero.alfa_s;
%     Vec_alfa = [alfa^2 alfa 1]';
%     
% end
% 
% if abs(beta) > PND.Aero.beta_m
%    
%     beta = PND.Aero.beta_m;
%     
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  Cm  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cm0     = Aero.Cm(1:3)*Vec_alfa; 
Cm_beta = Aero.Cm(4:6)*Vec_alfa;
Cm_p    = Aero.Cm(7:9)*Vec_alfa;
Cm_q    = Aero.Cm(10:12)*Vec_alfa;
Cm_r    = Aero.Cm(13:15)*Vec_alfa;
Cm_ail  = Aero.Cm(16:18)*Vec_alfa;
Cm_ele  = Aero.Cm(19:21)*Vec_alfa;
Cm_rud  = Aero.Cm(22:24)*Vec_alfa;

Cm      = Cm0 + Cm_beta*beta + Cm_p*p + Cm_q*q  + Cm_r*r + Cm_ail*delta_a + Cm_ele*delta_e  + Cm_rud*delta_r;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  Cn  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cn0     = Aero.Cn(1:3)*Vec_alfa; 
Cn_beta = Aero.Cn(4:6)*Vec_alfa;
Cn_p    = Aero.Cn(7:9)*Vec_alfa;
Cn_q    = Aero.Cn(10:12)*Vec_alfa;
Cn_r    = Aero.Cn(13:15)*Vec_alfa;
Cn_ail  = Aero.Cn(16:18)*Vec_alfa;
Cn_ele  = Aero.Cn(19:21)*Vec_alfa;
Cn_rud  = Aero.Cn(22:24)*Vec_alfa;

Cn      = Cn0 + Cn_beta*beta + Cn_p*p + Cn_q*q  + Cn_r*r + Cn_ail*delta_a + Cn_ele*delta_e  + Cn_rud*delta_r;   


%% Limiting moment coefficients to physical values:

% CX, CZ and Cm:

% if alfa > Aero.alfa_s*pi/180                                  % OLD: doubled conversion to radians - KY
%     [CX, ~, CZ, ~, Cm, ~] = Aero_Coefficients(Aero.alfa_s*pi/180,beta,p,q,r,Aero,delta_a,delta_r,delta_e);
%     disp('Stall limit! But no consequences (UNCOMMENT!!)')    % OLD: doubled conversion to radians - KY

%     [CX, ~, CZ, ~, Cm, ~] = Aero_Coefficients(PND.Aero.alfa_s*pi/180,beta,p,q,r,PND,delta_a,delta_r,delta_e);
%     disp('Stall limit!')

if alfa > Aero.alfa_s
    [CX, ~, CZ, ~, Cm, ~] = Aero_Coefficients(Aero.alfa_s,beta,p,q,r,Aero,delta_a,delta_r,delta_e);
%     disp('Stall limit! But no consequences (UNCOMMENT!!)')
    %disp('Stall limit! But no consequences (UNCOMMENT!!)')
end

% CY, Cl and Cn:

% if beta > Aero.beta_m*pi/180                                          % OLD: doubled conversion to radians - KY
%     [~, CY, ~, Cl, ~, Cn] = Aero_Coefficients(alfa,PND.Aero.beta_m*pi/180,p,q,r,PND,delta_a,delta_r,delta_e);
%     disp('Sideslip angle limit! But no consequences (UNCOMMENT!!)')   % OLD: doubled conversion to radians - KY

%     [~, CY, ~, Cl, ~, Cn] = Aero_Coefficients(alfa,PND.Aero.beta_m*pi/180,p,q,r,PND,delta_a,delta_r,delta_e);
%     disp('Sideslip angle limit! But no consequences (UNCOMMENT!!)')

if beta > abs(Aero.beta_m)
    [~, CY, ~, Cl, ~, Cn] = Aero_Coefficients(alfa,Aero.beta_m,p,q,r,Aero,delta_a,delta_r,delta_e);
    %disp('Sideslip angle limit!')
elseif beta < -abs(Aero.beta_m)
    [~, CY, ~, Cl, ~, Cn] = Aero_Coefficients(alfa,-Aero.beta_m,p,q,r,Aero,delta_a,delta_r,delta_e);
end

% % Cl:
% if Cl < -0.03
%    
%     Cl = -0.03;
%     disp('Cl limit exceeded!')
%     
% elseif Cl > 0.03
%     
%     Cl = 0.03;
%     disp('Cl limit exceeded!')
%     
% end
% 
% % Cm:
% if Cm < -0.4
%    
%     Cm = -0.4;
%     disp('Cm limit exceeded!')
%     
% elseif Cm > 0.4
%     
%     Cm = 0.4;
%     disp('Cm limit exceeded!')
%     
% end
% 
% % Cn:
% if Cn < -0.03
%    
%     Cn = -0.03;
%     disp('Cn limit exceeded!')
%     
% elseif Cn > 0.03
%     
%     Cn = 0.03;
%     disp('Cn limit exceeded!')
%     
% end
