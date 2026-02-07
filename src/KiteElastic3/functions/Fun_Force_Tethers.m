function [Fm, F_Att, M_Att, P_mass, FtetherUp, FtetherDown] = Fun_Force_Tethers(t, rm, vm, cv, rg, vg, omegaBE, REB, PD)

%--------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                        %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                    %
%             D. Nguyen, K. Yu                                            %
% Language  : Matlab                                                      %
% Synopsis  : Calculation of tether forces                                %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved %
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Inputs:  t       -> Time                                                %
%          rm      -> Particle position                                   %
%          vm      -> Particle velocity                                   %
%          cv      -> Control vector                                      %
%          rg      -> Kite position vectors (SE components)               % 
%          vg      -> Kite velocity vectors (SB components)               %
%          omegaBE -> Kite Angular velocities (SB components)             %
%          REB     -> Rotation matrix Earth - Body)                       % 
%                                                                         %                                             
%   Outputs:                                                              %
%          Fm      -> Force applied to each point mass                    %
%          F_Att   -> Force applied to the kite (body)                    %
%          M_Att   -> Torque applied to the kite (body)                   %
%          P_mass  -> Point mass weight                                   %
%          FtetherUp   -> Tether tension magnitude at the kite end        % 
%          FtetherDown -> Tether tension magnitude at the ground end      % 
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

RBE = REB';
nSegments   = PD.Tether.N;
g = PD.Env.g;
Bridles = PD.Bridle.Anchors;
Lengths = PD.Bridle.Lengths;
Connectivity = PD.Bridle.Connectivity;
nTethers = size(Connectivity,1);

eq_flag = PD.Bridle.Eq;
wind_type = PD.Env.Type;
wind_vw0 = PD.Env.vw;

r = reshape(rm,3*nSegments,[]);
v = reshape(vm,3*nSegments,[]);

CD = PD.Tether.CDt;
E   = PD.Tether.E; % Young modulus of the tethers
c0  = PD.Tether.C; % Damping of the tethers
A   = PD.Tether.A; % Cross area of the tether
rho = PD.Tether.rho; % Tether density

k0 = E*A; % Spring constant per segment (N), to be divided by element length

tetherRadius = sqrt(PD.Tether.A/pi);

Fs = zeros(size(r));
Fp = zeros(size(r));
Fd = zeros(size(r));
Fg = zeros(size(r));
P_mass = zeros(size(r));

PBE = RBE*(-rg);
TBE = [[RBE,PBE];[0,0,0,1]];
PEB = rg;
TEB = [[REB,PEB];[0,0,0,1]];

FsBridle = zeros(3,nTethers);
TsBridle = zeros(3,nTethers);
pMass = zeros(nTethers);
for i=1:1:nTethers
    pMass(i) = rho*cv(i)/(nSegments+1);
    for j=1:3:3*nSegments-2
        if (j == 3*nSegments-2)
            s = r(j:j+2,i) - PD.Anchor.Points(i,:)';
            sv = v(j:j+2,i);
            meanV = sum([v(j:j+2,i),[0;0;0]],2)/2;
            vw0 = Fun_Wind_Param(t,[0;0;0],wind_type,wind_vw0);
            vw1 = Fun_Wind_Param(t,r(j:j+2,i),wind_type,wind_vw0);
        else
            s = r(j:j+2,i) - r(j+3:j+3+2,i);
            sv = v(j:j+2,i) - v(j+3:j+3+2,i);
            meanV = sum([v(j:j+2,i),v(j+3:j+3+2,i)],2)/2;
            vw0 = Fun_Wind_Param(t,r(j+3:j+3+2,i),wind_type,wind_vw0);
            vw1 = Fun_Wind_Param(t,r(j:j+2,i),wind_type,wind_vw0);
        end
        air_den = PD.Env.ro;
        vw = sum([vw0,vw1],2)/2;

        sSetLength = cv(i)/nSegments;
        k = k0 / sSetLength;
        c = c0 / sSetLength;

        sActualLength = norm(s);

        Fs(j:j+2,i) = GetSegmentElasticForce(k, c, sActualLength, sSetLength,s,sv,eq_flag);
        Fd(j:j+2,i) = GetSegmentDragForce(vw,meanV,s,sActualLength,air_den,tetherRadius,CD);
    end
    
    FsBridle(:,i) = [0;0;0];
    TsBridle(:,i) = [0;0;0];
    bridlePointsB = zeros(3,length(Connectivity{i}));
    bridlePointsVB = zeros(3,length(Connectivity{i}));
    bridlePointsE = zeros(4,length(Connectivity{i}));
    bridlePointsVE = zeros(3,length(Connectivity{i}));
    bridlePointsS = zeros(3,length(Connectivity{i})); %Ojo con la dirección de S, pensar
    bridlePointsSv = zeros(3,length(Connectivity{i}));
    bridleSetLengths = zeros(1,length(Connectivity{i}));
    bridleActualLengths = zeros(1,length(Connectivity{i}));
    
    for j = 1:1:length(Connectivity{i})
        bridlePointsB(:,j) = Bridles(Connectivity{i}(j),:)';
        bridlePointsVB(:,j) = vg + cross(omegaBE,bridlePointsB(:,j));
        bridlePointsE(:,j) = (TEB*[bridlePointsB(:,j);1]);
        bridlePointsVE(:,j) = (RBE*bridlePointsVB(:,j));
        bridlePointsS(:,j) = bridlePointsE(1:3,j)-r(1:3,i);
        bridlePointsSv(:,j) = bridlePointsVE(1:3,j)-v(1:3,i);
        bridleSetLengths(j) = Lengths(Connectivity{i}(j));
        bridleActualLengths(j) = norm(bridlePointsS(:,j));
        k = k0 / bridleSetLengths(j);
        c = c0 / bridleSetLengths(j);
        FsBridlei = GetSegmentElasticForce(k, c, bridleActualLengths(j), bridleSetLengths(j),bridlePointsS(:,j),bridlePointsSv(:,j),eq_flag);
        FsBridle(:,i) = FsBridle(:,i) + FsBridlei;
        TsBridle(:,i) = TsBridle(:,i) + cross(bridlePointsB(:,j), RBE*FsBridlei);
    end
end

for i=1:1:nTethers
    for j=1:3:3*nSegments-2
        if j==1
            Fp(j:j+2,i) = -Fs(j:j+2,i) + pMass(i)*[0;0;g] + Fd(j:j+2,i)/2 + FsBridle(:,i);
        else
            Fp(j:j+2,i) = -Fs(j:j+2,i) + Fs(j-3:j-3+2,i) + pMass(i)*[0;0;g] + (Fd(j:j+2,i)+Fd(j-3:j-3+2,i))/2;
        end
    end
end

FtetherUp   = zeros(nTethers,1);
FtetherDown = zeros(nTethers,1);

jth = 3*nSegments-2;
for i=1:1:nTethers
    FtetherUp(i) = norm(FsBridle(:,i));
    FtetherDown(i) = norm(Fs(jth-3:jth-3+2,i));
end


Fm = reshape(Fp,[],1)';
for i=1:1:nTethers
    for j=1:3:3*nSegments-2
        P_mass(j:j+2,i) = pMass(i);
    end
end
P_mass = reshape(P_mass,[],1)';
F_Att = -RBE*sum(FsBridle,2);
M_Att = -sum(TsBridle,2); %comprobar signo
    
end

function Fd = GetSegmentDragForce(vWind,vs,s,sActualLength,air_den,tetherRadius,CD)
    va = vWind - vs;
    va = va - dot(va,s/sActualLength)*(s/sActualLength);
    Fd = CD*air_den*(sActualLength*2*tetherRadius)*norm(va)*va/2;
end

function Fg = GetSegmentGravityForce(sMass,g)
    Fg = [0;0;sMass*g];
end

function Fs = GetSegmentElasticForce(k, c, sActualLength, sSetLength,s,sv,eq_flag)
    if eq_flag == 0
        if (sActualLength - sSetLength >= 0)
            forceLimit = 1;
        else
            forceLimit = 0.001;
        end
    else
        forceLimit = 1;
    end
    Fs = forceLimit*(k*(sActualLength - sSetLength) + c*(dot(s/sActualLength,sv)))*(s/sActualLength);
end
