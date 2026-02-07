function [t, RBE, rg, vg, ag, Euler, omegaBE, omegaBE_p, Lambda, FT,...
          MT, FtetherUp, FtetherDown, FA, MA, W, alpha, beta, rm, vm,...
          am, cv, f] = Fun_ODE_Full_Output_KE3(t, xs, cv, PDext, PND)

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
% Inputs:  t   -> Time                                              %%
%          xs  -> Extended state vector                             %%
%                 [{rg} {vg} {q} {omegaBE} {rp} {vp}]               %%
%          cv  -> Control vector                                    %%
%          PDext -> Physical parameters                             %%
%          PD  -> Non-dimensional parameters                        %%
%   Outputs:                                                                 %
%           t        -> Dimensional time                                     % 
%           RBE      -> Kite-Earth Rotation matrices                         % 
%           rg       -> Kite position vectors (SE components)                % 
%           vg       -> Kite velocity vectors (SB components)                %
%           ag       -> Kite acceleration in Earth frame (dvg/dt|E)          %
%           Euler    -> Euler angles                                         % 
%           omegaBE  -> Kite Angular velocities (SB components)              %
%           omegaBE_p-> Kite angular acceleration in Earth frame             %
%           Lambda   -> Tether Tensions along tether directions              % 
%           FT       -> Tether Forces upon the kites                         % 
%           MT       -> Tether torque upon the kites                         %
%           FtetherUp   -> Tether tension magnitude at the kite end          % 
%           FtetherDown -> Tether tension magnitude at the ground end        % 
%           FA       -> Aerodynamic force upon the kites                     % 
%           MA       -> Aerodynamic force upon the kites                     % 
%           alpha    -> Angles of attack of the kites                        % 
%           beta     -> Sideslip angles of the kites                         % 
%           rm       -> Position vectors of the masses (SE components)       % 
%           vm       -> Velocity of the masses (SE components)               %
%           am       -> Masses acceleration in Earth frame                   %
%           cv       -> Control vector                                       %
%           f        -> Time derivative of the extended state vector         % 
%                                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Preliminaries:

PD = PDext;

% This block is for debugging and changing values from PD in a stable
% manner
% persistent x_prev a counter
% if isempty(x_prev)
%     a = 1;
%     x_prev = xs(2);
%     counter=0;
% end
% if x_prev<0 && xs(2) >=0 && counter<25
%     a = a - 0.001;
%     counter = counter + 1;
%     fprintf('Changing PD at %3.2f s\n', t);
% end
% x_prev = xs(2);

% Saving time for UnPaM aerodynamics:
PD.Aero.t = t; 

% Retrieving parameters:
% Kite:
mk     = PD.Kite.m;
IG     = PD.Kite.IG;
IG_inv = PD.Kite.IG_inv;

% Tether:
L0  = PD.Tether.L0;
rho = PD.Tether.rho;
N   = PD.Tether.N;

nTethers = length(PD.Tether.L0);
nSegments   = PD.Tether.N;

mm  = zeros(1 , (nTethers-1)*nSegments + nSegments);   % initialise codegen - KY
mm3 = zeros( 3 * nTethers * nSegments, 1);            % initialise codegen - KY

% Lenght of each segment
for i = 1:nTethers
    for j = 1:nSegments
        mm((i-1)*nSegments + j) = (rho*L0(i))/nSegments;
    end
    mm(i * nSegments) = 0.5*(rho*L0(i))/nSegments;
end

for i=1:length(mm)
    mm3(3*(i-1)+1:3*(i-1)+1+2,1) = [mm(i),mm(i),mm(i)];
end

% Getting dimension of the dynamical system:
n = length(xs);

% Variables pre-allocation:
f = zeros(n,1);

% Spliting state vector in kinematic vectors:
rg = xs(1:3); % Kite CG position expressed in Earth frame
vg = xs(4:6); % Kite CG velocity expressed in body frame
q = xs(7:10); % Kite quaternions
omegaBE = xs(11:13); % Kite angular velocity expressed in body frame
rm_f = 14;
rm_e = 13+(n-13)/2;
vm_f = rm_e + 1;
vm_e = vm_f - 1 + (n-13)/2;
rm = xs(rm_f:rm_e); % Point masses positions expressed in Earth frame [{rp1}, {rp2}, ...,{rpN}, {rm1}, {rm2}, ..., {rmN}]. 3 components per r
vm = xs(vm_f:vm_e); % Point masses velocities expressed in Earth frame (same order as [rm]). 3 components per v

%% Kinematic precomputations:

% Getting Euler anges from quaterions:
Euler = quat2eul(q','ZYX'); % Tait-Bryan criterion
% psi = Euler(1);
the = Euler(2)*180/pi;
% phi = Euler(3);

the_S_Eq = atan2(-xs(3),-xs(1))*180/pi;

qnorm = norm(q);
qw = q(1)/qnorm;
qx = q(2)/qnorm;
qy = q(3)/qnorm;
qz = q(4)/qnorm;

% Rotation matrices:

REB = [1 - 2*qy^2 - 2*qz^2,  2*qx*qy - 2*qw*qz,    2*qx*qz + 2*qw*qy;
        2*qx*qy + 2*qw*qz,    1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qw*qx;
        2*qx*qz - 2*qw*qy,    2*qy*qz + 2*qw*qx,    1 - 2*qx^2 - 2*qy^2];

RBE = REB';


%% Aerodynamic force and torque about kite CG:

% Wind speed components in the Earth Frame:
vw = Fun_Wind(t,rg,PD); % Tricking function by inputing PD instead of PND (results in m/s!)

% Wind velocity in the body frame:
vw = RBE*vw;

% Components of the aerodynamic velocity vector in the body frame:
va = vg - vw;

% Components in the body frame of the aerodynamic force and torque:
[FA, MA, alpha, beta] = Aerokite(va,omegaBE,PD,PND,0,0,0); % Tricking function by inputing PD instead of PND (results with dimensions!)
% [FA, MA, alpha, beta] = Aerokite_UnPaM(t,va,omegaBE,PD,0,0,0); % UnPaM aerodynamics
% [FA, MA, alpha, beta] = Aerokite_QS(va,omegaBE,PD,PND,0,0,0); % QS aerodyanamics

%% Weight force:
% Weight force on the kite in the Earth frame:
W = [0; 0; 1]*mk*PD.Env.g;

% Weight force in the Body frame:
FW = RBE*W;

%% Tethers forces and torque about kite CG:
% Computing the force felt by every particle in Earth frame (sorted as [rm] and [vm]):
[Fm, FT, MT, P_mass, FtetherUp, FtetherDown] = Fun_Force_Tethers(t, rm, vm, cv, rg, vg, omegaBE, REB, PD);

%% Total forces and moments:
% Total force on the kite CG in body frame:
Fk = FA + FT + FW; % Aerodynamic, tension and weight forces

% Total torque about the kite CG in body frame:
Mk = MA + MT; % Aerodynamic and tension torques

%% Equations of motion of the kite:

% Kinematic relation (drg/dt|E = REB*vg|B):
f(1:3) = REB*vg;

% Newton 2nd law for the kite (including Coriolis):
f(4:6) = Fk/mk - cross(omegaBE,vg);

% Quaternions propagation ('q' order must be [w x y z]):
A = [ 0,           -omegaBE(1), -omegaBE(2), -omegaBE(3);
      omegaBE(1),  0,           omegaBE(3),  -omegaBE(2);
      omegaBE(2),  -omegaBE(3), 0,           omegaBE(1);
      omegaBE(3),  omegaBE(2),  -omegaBE(1), 0 ];
f(7:10) = 1/2*A*q;

% Angular momentum equation:
f(11:13) = IG_inv*(Mk - cross(omegaBE,IG*omegaBE));


%% Equations of motion of the tether mass points:

% Kinematic relations (drm/dt|E = vm|E):
f(rm_f:rm_e) = vm;

% Newton 2nd law for each point mass:
f(vm_f:vm_e) = Fm./P_mass;


%% Computing & Transforming some output magnitudes:
      
% Kite acceleration in Earth frame (dvg/dt|E):
ag = Fk/mk;
ag = REB*ag;

% Kite angular acceleration (domegaBE/dt|E) in Earth frame:
omegaBE_p = IG_inv*Mk;
omegaBE_p = REB*omegaBE_p;

% Tether tension magnitudes:
Lambda = FtetherUp;

% Masses acceleration in Earth frame:
am = Fm./mm3';

end
