function [T, u, cv, icv] = RK4Integrator_KE3(t_end,t_inc, u0, cv0, PDext)

PD = PDext;
PND = Fun_PND_KA(PD);
%PND = Aero_Parameters_QS(PD,PND);

tspan = 0:t_inc:t_end;
%% Preliminaries:

% tstep.timeStepTimeout = 10; % disabled for codegen - KY
% Variables pre-allocation:
u = zeros(length(tspan),length(u0));   % state vector
cv = zeros(length(cv0),length(tspan)); % control vector
icv = zeros(12,length(tspan));         % Internal controller state vector

% Defining alpha for final computation of state vector (DON'T TOUCH!):
alpha = 1/6*[1, 2, 2, 1];

% Setting initial conditions:
u(1,:) = u0;

k1 = zeros(length(u0),1);
k2 = zeros(length(u0),1);
k3 = zeros(length(u0),1);
k4 = zeros(length(u0),1);

% Assigning output time:
T = tspan;

% Creating warning if too big Dt:
Vlongwaves = sqrt(PD.Tether.E/(PD.Tether.rho/PD.Tether.A));
Dt_min     = 1/5*(PD.Tether.L0/PD.Tether.N)/Vlongwaves; % Minimum Dt to catch longitudinal waves

if T(2) - T(1) > Dt_min
    disp('Warning! Time step in RK4Integrator is too big')
end


%% Integrating along tspan:
% It starts at time 2 since, at time 1, the values are the IC's directly;

Count = 0;
cv(:,1) = cv0;

for i = 2:length(tspan)
    progress = 100 * T(i)/T(end);
    if abs(mod(progress, 5)) < 1e-10
        fprintf('Simulation progress: %3.0f%% (%3.2f / %3.2f seconds)\n', ...
                round(progress), T(i), T(end));
    end
    
    % Computing dt:
    dt = T(i)-T(i-1);
    deltaT = dt;

    % Compute control
    [cv(:,i), icv(:,i)] = Fun_Control_KE3(T(i-1),PD,u(i-1,:)',cv(:,i-1),deltaT);

    % UnPaM code (disabled)
%     xoverFcn = @(t,y)  myEventsFcn(t,y,tstep);                                            % codegen - KY
%     options = odeset('RelTol',PD.Num.RelTol,'AbsTol',PD.Num.AbsTol,'Events',xoverFcn);    % codegen - KY
%     f = @(t,u0) Fun(t,u0,cv(:,i));                                                        % codegen - KY

        % Prepare info for UnPaM's time step:
        PD.Aero.FF = 0;  % Update UnPaM wake
            
        % Running stages:
        % Stage 1:
        if ceil(T(i)/PD.Aero.DT) > Count % Running UnPaM every some DT (holding previous forces/moments)
    
            PD.Aero.Flag_TS = 1;  % Run 1 time step of aerodynamics with UnPaM
            
            Count = Count + 1;
            
%           f(T(i-1),u(i-1,:)'); % Run 1 time step for UnPam  (DISABLED)
            PD.Aero.Flag_TS = 0; 

        else
            PD.Aero.Flag_TS = 0; % Don't run 1 time step (keep aerodynamics constant)
        end

        % [~,y] = ode15s(f,[T(i-1) T(i)], u(i-1,:)',options);
        % u(i,:) = y(end,:);

        sanityCheck = true;
%         k1 = feval(Fun, T(i-1)       , u(i-1,:)', cv(:,i)); % UnPaM (disabled)
        k1 = Fun_ODE_KE3(T(i-1), u(i-1,:)', cv(:,i), PD, PND);

        if anynan(k1)
            sanityCheck = false;
        end

        % Stage 2:
        if sanityCheck
            PD.Aero.Flag_TS = 0; % Retrieve previous data from UnPaM
    %         k2 = feval(Fun, T(i-1)+0.5*dt, u(i-1,:)'+0.5*dt*k1, cv(:,i)); % UnPaM (disabled)
            k2 = Fun_ODE_KE3(T(i-1)+0.5*dt, u(i-1,:)'+0.5*dt*k1, cv(:,i), PD, PND);
            if anynan(k2)
                sanityCheck = false;
            end
        end

        % Stage 3:
%         k3 = feval(Fun, T(i-1)+0.5*dt, u(i-1,:)'+0.5*dt*k2, cv(:,i)); % UnPaM (disabled)
        if sanityCheck
            PD.Aero.Flag_TS = 0; % Retrieve previous data from UnPaM
            k3 = Fun_ODE_KE3(T(i-1)+0.5*dt, u(i-1,:)'+0.5*dt*k2, cv(:,i), PD, PND);
            if anynan(k3)
                sanityCheck = false;
            end
        end

        % Stage 4:
        if sanityCheck
%         k4 = feval(Fun, T(i-1)+dt    , u(i-1,:)'+dt*k3, cv(:,i)); % UnPaM (disabled)
            k4 = Fun_ODE_KE3(T(i-1)+dt, u(i-1,:)'+dt*k3, cv(:,i), PD, PND);
            if anynan(k4)
                sanityCheck = false;
            end
        end

        % Computing the final value of the state vector at time-step 'i':
        if sanityCheck
            k = [k1'; k2'; k3'; k4'];
            u(i,:) = u(i-1,:) + dt*alpha*k;
        else
            fprintf(1,'NaN values detected at %3.5f s, aborting\n',T(i))
            T      = T(1:i-1);
            u      = u(1:i-1,:);
            cv     = cv(:,1:i-1);
            icv    = icv(:,1:i-1);
            break
        end

% Disabled (UnPaM)
%     % Saving kinematics and coefficients into UnPaM global variable:     % codegen - KY
%     if PD.Aero.UnPaM == 1 || PD.Aero.UnPaM == 2                          % codegen - KY
%         UnPaM(i,:) = UnPaM_A;                                            % codegen - KY
%         save(PD.Aero.NamePost, 'T', 'u', 'PD', 'UnPaM', 'Ctr')           % codegen - KY
%     end                                                                  % codegen - KY
    
%     % Saving control variables and their derivatives into Ctr global
%     % variable:
%     if PD.Ctr.Type == 5 || PD.Ctr.Type == 6 % Feedback control
%        
%         Ctr(i,:) = Ctr_A;
%         
%     end

end



end
% function [value,isterminal,direction] = myEventsFcn(t,y,tstep)
%     lastTimeStepTime = tstep.lastTimeStepTime;
%     timeStepTimeout  = tstep.timeStepTimeout;
%     value = seconds(timeStepTimeout)-(datetime("now")-lastTimeStepTime);
%     isterminal = 1;
%     direction = 0;
% end
