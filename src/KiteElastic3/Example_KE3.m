%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Example of main file                                           %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------

%% Initialization
% Must clear workspace before running as persistent declarations are used
close all; clear all; clc;

addpath('common','functions');

eq_flag = false; % if true, search for equilibrium and run simulation
                 % if false, load the savestate in the middle of the
                 %    figure-eight.

animation_flag = true; % if true, plot a 3D animation of the flight

% Paremetrized initial conditions
p0 = [7.2, ...      Wind velocity
      0.1,...       Ctr.delta1 (unused)
      0.5,...       Ctr.om_delta1 (unused)
      2,...         Kite mass
      [100,100,100.535],... Initial tether lenght 
      ];
% Proposed initial tether lenghts:
% [100,100,100.535] → v_w = 7.2
% [100,100,100.565] → v_w = 5.4

PD = Fun_PD_RFD(p0);
PND = Fun_PND_KA(PD);

if PD.Aero.ModelSwitch == 2 % experimental, not functional
    addpath('functions_QS');
    PND = Aero_Parameters_QS(PD,PND); 
end

%% Equilibrium search
if eq_flag
    [u0, Error, Flag] = Equilibrium_New_KA(0,PD,PND);
    
    if Flag == 0
        disp('Equilibrium not found!')
    end
        
    u0 = u0';
    cv0 = PD.Tether.L0;

    % Checking pitching angle (AoA in equilibrium conditions) and spherical 
    % elevation for validation purposes:
    Euler_Eq = quat2eul(u0(7:10));
    the_Eq = Euler_Eq(2)*180/pi; % In degrees
    the_S_Eq = atan(u0(3)/u0(1))*180/pi; % In degrees
    
    fprintf('Equilibrium at - Euler theta: %3.2f / Lambda: %3.2f\n',the_Eq,the_S_Eq);
    
    % Stability Analysis:
    disp('---------------------------------------------------------')
    disp('Stability Analysis of the Equilibrium State')
    Jnum        = Jacobian('Fun_ODE_KE3',0,u0',PD,PND,cv0);
    [Vec, Val]   = eig(Jnum);
    
    figure(73)
    plot(real(Val),imag(Val),'+')
    xlabel('Real(\lambda)','fontsize',12)
    ylabel('Imag(\lambda)')
    set(gca,'box','on','fontsize',12)
end

%% Integration
if ~eq_flag
    % Add restart points here to start from figure-eight.
    %   Examples provided: 5.4 m/s & 7.2 m/s
    if PD.Env.Vw == 5.4
        load('./foe100_54.mat');
    elseif PD.Env.Vw == 7.2
        load('./foe100_72.mat');
    else
        error('No avaliable figure-eight restart point for specified wind speed. The simulation must start from equilibrium.')
    end
else
    % If we start from equilibrium, the following sequence will be
    % executed:
    %   (1) Perturb equilibrium.
    %   (2) Restore equilibrium.
    %   (3) Perform descend sequence.
    %   (4) Figure-eight 
    % The pre-defined descend sequence has been tested for 7.2 m/s. Other
    % wind speeds may be unstable or require tuning at Fun_Control_States.m
    % to work.
end

% Integrate in time:
deltaT = 1e-4; 
t_end  = 60;

% (!) RK4Integrator_KE3 is the standard Matlab version. Good for debugging
% but very slow. We recommend using the RK4Integrator_KE3_mex version,
% which is compiled using Matlab Coder. If you make any changes to the
% underlying code, you must regenerate the .mexw64 file. Uncomment as
% needed.

% WARNING: RK4Integrator_KE3_mex has been compiled with R2025b. If you are
% running an older version of Matlab and execution fails, you may have to
% regenerate it using Matlab Coder on the RK4Integrator_KE3 function.

%[T, u, cv, icv] = RK4Integrator_KE3(t_end,deltaT, u0, cv0, PD); % Matlab
[T, u, cv, icv] = RK4Integrator_KE3_mex(t_end,deltaT, u0, cv0, PD); % mexw64 (compiled)

%% Post processing
k = 1;
postProcessTimeStepSeconds = 0.01;
postProcessStartTime = 0+1e-12;
postProcessQueryRange = ceil(postProcessStartTime/deltaT):floor(length(T)/(T(end)/postProcessTimeStepSeconds)):length(T);
for i = postProcessQueryRange
    if floor(T(i))==T(i)
        fprintf('Postprocess progress: %u/%u seconds (%3.2f%%)\n',T(i),T(end),100*T(i)/T(end))
    end
    [Tout(k), RBE(:,:,k), rk(:,k), vk(:,k), ak(:,k), Euler(:,k),...
        omegaBE(:,k), omegaBE_p(:,k), Lambda(:,k), FT(:,k),...
        MT(:,k), FtetherUp(k,:), FtetherDown(k,:), FA(:,k), MA(:,k),...
        FW(:,k), alpha(k), beta(k), rm(:,k), vm(:,k),...
        am(:,k), ~, f(:,k)] = Fun_ODE_Full_Output_KE3(T(i), u(i,:)', cv(:,i), PD, PND);
        cv_out(:,k) = cv(:,i);
        icv_out(:,k) = icv(:,i);
    k = k + 1;
end

%% 3D animation plot
if animation_flag
    if ishandle(23); close(23); end
    
    speedRate = 2;  % 1.0 → Real time // <1.0 → Slow-mo // >1.0 → Fast-forward
    
    animationHandles = Plot_playKiteAnimation(Tout, u, RBE, PD, postProcessQueryRange, speedRate);
end

%% 2D plots
Flag_Plot = [1 1 1 1 1 1 1 1 1]; % [Position Velocity Euler alfa&beta Tensions Lengths AeroForces AeroTorques Controller]

Plot_Results_KE3(Tout, RBE, rk, vk, ak, Euler, omegaBE, omegaBE_p, Lambda, FT, MT, FtetherUp, FtetherDown, FA, MA, FW, alpha, beta, rm, vm, am, cv(:,postProcessQueryRange), icv(:,postProcessQueryRange), T, icv, Flag_Plot)

%% Functions
function handles = Plot_playKiteAnimation(Tout, u, RBE, PD, postProcessQueryRange, speedRate)
    handles = Plot_initKiteAnimation(PD);
    
    simStart   = Tout(1);
    timesList  = Tout - simStart;
    nFrames    = numel(timesList);
    
    tic;
    nextFrame = 1;
    while nextFrame <= nFrames
        tWall = toc;
        
        desiredWallNext = timesList(nextFrame)/speedRate;
        dtWait = desiredWallNext - tWall;
        if dtWait > 0
            pause(dtWait);
            tWall = toc;
        end
        
        tSimDesired = tWall * speedRate;
        idxs = nextFrame:nFrames;
        rel  = find(timesList(idxs) >= tSimDesired, 1);
        if isempty(rel)
            break;   % we've played to the end
        end
        k = idxs(rel);   % absolute frame index in 1:nFrames
        
        % ---- actually draw frame k ----
        simIdx = postProcessQueryRange(k);
        tSim   = timesList(k);
        Plot_updateFrame(handles, tSim, u(simIdx,:), RBE(:,:,k), PD);
        drawnow;
        
        % ---- advance to the next frame after k ----
        nextFrame = k + 1;
    end
end
