function u0 = Fun_Gen_ICs(the_S0, vg0, Euler0, omegaBE0, PD)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga                        %
% Language  : Matlab                                                         %
% Synopsis  : Generate initial condition for equilibrium search              %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------

%% Preliminaries:

% Retrieving parameters:

Anchors = PD.Bridle.Anchors;
Lengths = PD.Bridle.Lengths;
Connectivity = PD.Bridle.Connectivity;
nTethers = size(Connectivity,1);
EarthAnchors = PD.Anchor.Points';

nSegments   = PD.Tether.N;
L0  = PD.Tether.L0;

% Variables pre-allocation:
u0 = zeros(13+3*2*nTethers*nSegments,1);

% Rotation matrix:
phi = Euler0(1);
the = Euler0(2);
psi = Euler0(3);

RBE = EulerConstructRBE(phi, the, psi);
REB = RBE';


%% Solving nonlinear algebraic problem to get kite CG position:
% We need to respect that the 2 tethers length are L0, the 3 Euler angles
% and the inclination the_S0 given as an input (6 dof in total);

% Initial guess for two dof left (to define position in spherical coord.):
X_IG = [mean(L0); 0]; % [rg module, phi_S]

% Solve nonlinear algebraic problem:
if nTethers == 3
    [X,~,Flag] = fsolve(@(X) ThreeLineU0(X), [mean(L0); 0; the]);

    rg_mod = X(1); % Module of kite position vector
    phi_S0 = X(2); % Kite azimuthal angle
    the = wrapTo2Pi(X(3));
    RBE = EulerConstructRBE(phi, the, psi);
    REB = RBE';

elseif nTethers == 2
    [X, ~, Flag] = my_fzero(@(t, X) TwoLineU0(t, X),  [mean(L0); 0], PD);
    rg_mod = X(1); % Module of kite position vector
    phi_S0 = X(2); % Kite azimuthal angle
else
    error("Unsupported number of tethers")
end

if Flag == 0
   
    error('Nonlinear system for position ICs not solved in "Fun_Gen_ICs"!!')
    
end

% Computing kite CG position vector after convergence:
rg0 = rg_mod*[-cos(the_S0)*cos(phi_S0); cos(the_S0)*sin(phi_S0); -sin(the_S0)];


%% Assigning rigid body part:

u0(1:3)   = rg0; % Position
u0(4:6)   = vg0; % Velocity
% u0(7:10)  = eul2quat(Euler0', 'ZYX'); % Quaternions
u0(7:10)  = eul2quat([phi,the,psi], 'ZYX'); % Quaternions
u0(11:13) = omegaBE0; % Angular rate body-Earth

%% Assigning point mass position and velocity:
% Position considering straight tethers from F (control system) to A
% (bridle attachment point);
% Velocity considering linear profile of velocities from point A (V max) to
% point F (V = 0);

% Preparation:

[couplingPointsB, couplingPointsE] = Fun_GetCouplingPointsU0(nTethers, Connectivity, Anchors, Lengths, RBE, EarthAnchors, rg0);

tetherDirections = (couplingPointsE-EarthAnchors) ./ vecnorm(couplingPointsE-EarthAnchors,2);

for i = 1:1:nTethers
    couplingPointsVelocity(:,i) = REB*(vg0 + cross(omegaBE0,couplingPointsB(i,:)'));
end

index = 13+1;
for i = 1:nTethers
    for j = 1:nSegments
        u0(index:index+2) = EarthAnchors(:,i) + L0(i)*(1-(j-1)/nSegments)*tetherDirections(:,i)*(1-1e-4);
        index = index+3;
    end
end
for i = 1:nTethers
    for j = 1:nSegments
        u0(index:index+2) = (1-(j-1)/nSegments)*couplingPointsVelocity(:,i);
        index = index+3;
    end
end


%% Internal functions:
    function F = ThreeLineU0(X)
        rg_mod0 = X(1);
        phi_S0 = X(2);
        the0 = X(3);
        
        rg0 = rg_mod0*[-cos(the_S0)*cos(phi_S0); cos(the_S0)*sin(phi_S0); -sin(the_S0)];
        RBE = EulerConstructRBE(phi, the0, psi);
        REB = RBE';
        [~,couplingPointsE] = Fun_GetCouplingPointsU0(nTethers, Connectivity, Anchors, Lengths, RBE, EarthAnchors, rg0);
        lineLengths = vecnorm(couplingPointsE-EarthAnchors,2);
        F = L0 - lineLengths';
    end

    function F = TwoLineU0(t,X)
        rg_mod0 = X(1);
        phi_S0 = X(2);
        
        rg0 = rg_mod0*[-cos(the_S0)*cos(phi_S0); cos(the_S0)*sin(phi_S0); -sin(the_S0)];
        RBE = EulerConstructRBE(phi, the, psi);
        REB = RBE';
        [~,couplingPointsE] = Fun_GetCouplingPointsU0(nTethers, Connectivity, Anchors, Lengths, RBE, EarthAnchors, rg0);
        lineLengths = vecnorm(couplingPointsE-EarthAnchors,2);
        F = L0 - lineLengths';
    end

    function RBE = EulerConstructRBE(phi, the, psi)
        RBE = [ cos(the)*cos(psi),                            cos(the)*sin(psi),                            -sin(the);
            sin(phi)*sin(the)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(the)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(the);
            cos(phi)*sin(the)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(the)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(the) ];
    end
    
    function [couplingPointsB, couplingPointsE] = Fun_GetCouplingPointsU0(nTethers, Connectivity, Anchors, Lengths, RBE, EarthAnchors, rg0)
        couplingPointsB = zeros(nTethers,3);
        REB = RBE';
        for i = 1:1:nTethers
            for j = 1:1:length(Connectivity{i})
                anchorPoints(j,:) = Anchors(Connectivity{i}(j),:);
                lengths(j) = Lengths(Connectivity{i}(j));
            end
            couplingPointsB(i,:) = Fun_CalculateCouplingPoint(anchorPoints,lengths,-RBE*rg0+RBE*EarthAnchors(:,i));
            clear anchorPoints lengths
        end
        couplingPointsE = rg0 + REB*couplingPointsB';
    end

    function [CouplingPoint] = Fun_CalculateCouplingPoint(Anchors,Lengths,ReferencePoint)
        DefaultDir = (ReferencePoint' - mean(Anchors,1))';
        DefaultDir = DefaultDir/norm(DefaultDir);
        options = optimset('Display','off');
        if length(Lengths) == 1
            CouplingPoint = Anchors + Lengths*DefaultDir'/norm(DefaultDir);
        elseif length(Lengths) == 2
            CouplingPoint = fsolve(@(x)TwoPointEq(x,Anchors,Lengths,DefaultDir),mean(Anchors)+[0,0,1],options);
        elseif length(Lengths) == 3
            CouplingPoint = fsolve(@(x)ThreePointEq(x,Anchors,Lengths),mean(Anchors)+[0,0,1],options);
        else
            disp("Overdetermined system: Iterative search for tensioned triad") %TODO
    
        end
    end
    
    function F = TwoPointEq(x,Anchors,Lengths,DefaultDir)
        F(1) = -Lengths(1) + norm(Anchors(1,:)-x);
        F(2) = -Lengths(2) + norm(Anchors(2,:)-x);
        n = cross(Anchors(2,:)-Anchors(1,:),Anchors(2,:)+DefaultDir'-Anchors(1,:));
        n = n/norm(n);
        k = -n(1)*Anchors(1,1)-n(2)*Anchors(1,2)-n(3)*Anchors(1,3);
        F(3) = n(1)*x(1) + n(2)*x(2) + n(3)*x(3) + k;
    end
    
    function F = ThreePointEq(x,Anchors,Lengths)
        F(1) = -Lengths(1) + norm(Anchors(1,:)-x);
        F(2) = -Lengths(2) + norm(Anchors(2,:)-x);
        F(3) = -Lengths(3) + norm(Anchors(3,:)-x);
    end

end
