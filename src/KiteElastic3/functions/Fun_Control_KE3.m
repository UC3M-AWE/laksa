function [cv, icv] = Fun_Control_KE3(t,PD,xs,cv_0,deltaT,reelout_speed)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga,                       %
%             D. Nguyen, K. Yu                                               %
% Language  : Matlab                                                         %
% Synopsis  : Generate control inputs                                        %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %%
% Inputs:  t               -> time                                         %%
%          PD              -> Parameters                                   %%
%          xs              -> state vector                                 %%
%          cv_0            -> initial/previous control vector              %%
%          deltaT          -> simulation timestep                          %%
%          reelout_speed   -> Current reelout speed                        %%
%                                                                          %%
% Outputs: cv              -> control vector                               %%
%          icv             -> Internal controller state vector             %%
%                                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Documentation:
%  DeLosRíos-Navarrete, J. González-García, I. Castro-Fernández, et al., "A
% small-scale and autonomous testbed for three-line delta kites applied to airborne
% wind energy," Wind Energy Science, vol. 10, no. 6, pp. 1153–1166, 2025-06. doi:
% 10.5194/wes-10-1153-2025. [Online].
% Available: https://wes.copernicus.org/articles/10/1153/2025/

if nargin == 5
    reelout_speed = 0;
end

% codegen declarations
icv = zeros(12,1); % initialise codegen - KY
psi_set = 0;
cv = zeros(3,1);
coder.extrinsic("Fun_Control_States");
controller_parameters = struct;
controller_parameters.waypoint_type = 'none';
controller_parameters.waypoint_type = 'long placeholder so matlab coder gives a proper size limit to this string';
controller_parameters.waypoint = [0,0];
controller_parameters.gains = [0,0,0];

persistent icv_0 cv_init                          % state and control output
persistent psi_0 turns                            % navigation
persistent current_state state_prev               % guidance
persistent psi_0_filtered ITerm DeltaL_u_0        % control
persistent t_last_controller_cycle t_last_iter    % internal variables
persistent reelout_length                         % reel-out
persistent DeltaL_power_0                         % pitching
persistent y_last_iter

if isempty(cv_init) || t == 0
    cv = cv_0; % initialize control vector
    y_last_iter = xs(2);
    cv_init = PD.Tether.L0;

    current_state = 'long placeholder so matlab coder gives a proper size limit to this string';

    if cv_init(1)-cv_0(1) == 0
    % if the tether is not actuated, simulation from equilibrium is assumed
        current_state = 'open-loop_zenith-test';
    else
    % otherwise, this is a resumed simulation
        current_state = 'foe_goright';
    end
    state_prev = current_state;

    % simulate 10 seconds of actuation to initialize actuators
    actual_DeltaL_u_L = []; actual_DeltaL_u_R = []; L_act_pos = []; R_act_pos = [];
    for i=1:ceil(10/deltaT)
       [actual_DeltaL_u_L, actual_DeltaL_u_R, L_act_pos, R_act_pos] = Fun_Actuators((cv_0(2)-cv_init(2))+(cv_init(1)-cv_0(1)),0,1,deltaT);
    end

    % initialize variables
    DeltaL_u_0 = (cv_0(2)-cv_init(2))+(cv_init(1)-cv_0(1));
    ITerm = 0;
    psi_0_filtered = NaN;
    psi_0 = 0;
    icv_0 = icv;
    turns = 0;
    reelout_length = 0;
    DeltaL_power_0 = 0;

    icv = writeControllerStateVector(NaN, NaN, NaN, NaN, NaN, ...
        NaN, NaN, L_act_pos, R_act_pos, actual_DeltaL_u_L + actual_DeltaL_u_R,...
        (cv_0(2)-cv_init(2))+(cv_init(1)-cv_0(1)), NaN);

    t_last_iter = t;
    t_last_controller_cycle = 0;


elseif ((t~=t_last_iter) && (t-t_last_controller_cycle > 1/PD.Ctr.Rate))
    y_last_iter = xs(2);
    % new controller cycle

    %%%%%%%%%%%%%%%%%%
    %%% Navigation %%%
    %%%%%%%%%%%%%%%%%%
    q = xs(7:10); % Kite quaternions
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

    % kite position vector
    rk = xs(1:3);
    rk(1) = -rk(1);
    rk(3) = -rk(3);
    % kite spine vector
    i_k = REB*[1,0,0]';
    i_k(1) = -i_k(1);
    i_k(3) = -i_k(3);

    % Azimuth (delta) and elevation (lambda)
    delta = atan2d(rk(2),rk(1));
    lambda = atan2d(rk(3),sqrt(rk(1)^2 + rk(2)^2));
    
    % Heading angle (psi)
    psi = atan2d(dot(i_k,[sind(delta),-cosd(delta),0]'),...
                 dot(i_k,[-sind(lambda)*cosd(delta),-sind(lambda)*sind(delta),cosd(lambda)]'));
    psi = manual_wrapTo180(psi);

    if angleQuadrant(psi) == 2 && angleQuadrant(psi_0) == 3
        turns = turns-1;
    elseif angleQuadrant(psi) == 3 && angleQuadrant(psi_0) == 2
        turns = turns+1;
    end
    psi = psi + turns*360;  
    psi_0 = psi;

    % Geometric DeltaL

    DeltaL_g = (PD.Anchor.Points(1,2) + (-PD.Anchor.Points(2,2)))*sind(delta)*cosd(lambda);

    %%%%%%%%%%%%%%%%
    %%% Guidance %%%
    %%%%%%%%%%%%%%%%
    [current_state,controller_parameters] = Fun_Control_States(t,psi,delta,lambda,current_state);

    % Determine psi_set
    switch controller_parameters.waypoint_type
        case "attractor"
            phi_set = controller_parameters.waypoint(1);
            beta_set = controller_parameters.waypoint(2);

            y = -sind(phi_set-delta)*cosd(beta_set);
            x = cosd(lambda)*sind(beta_set) - sind(lambda)*cosd(beta_set)*cosd(phi_set-delta);
            psi_set = atan2d(y,x);
            psi_set = manual_wrapTo180(psi_set);
        case "centre"
            lobe_center_phi = controller_parameters.waypoint(1);
            lobe_center_beta = controller_parameters.waypoint(2);

            y = -sind(delta-lobe_center_phi)*cosd(lobe_center_beta);
            x = -cosd(lambda)*sind(lobe_center_beta) + sind(lambda)*cosd(lobe_center_beta)*cosd(delta-lobe_center_phi);
            psi_set = atan2d(y,x) + sign(delta)*90;
            psi_set = manual_wrapTo180(psi_set);
    end

    %%%%%%%%%%%%%%%%%%
    %%% Controller %%%
    %%%%%%%%%%%%%%%%%%
    switch controller_parameters.waypoint_type
        case 'fixed_input'
            DeltaL_u = controller_parameters.waypoint(1);
            DeltaL_power = controller_parameters.waypoint(2);

            [actual_DeltaL_u_L, actual_DeltaL_u_R, L_act_pos, R_act_pos] = Fun_Actuators(DeltaL_u,DeltaL_power,1,deltaT);

            cv = cv_init+[-actual_DeltaL_u_L;actual_DeltaL_u_R;0];

            icv = writeControllerStateVector(0, 0, 0, delta, lambda, ...
                NaN, psi, L_act_pos, R_act_pos, actual_DeltaL_u_L + actual_DeltaL_u_R,...
                DeltaL_u, DeltaL_g);
            icv_0 = icv;


        case {'attractor','centre'}
            k = controller_parameters.gains(1);
            ki = controller_parameters.gains(2);
            kd = controller_parameters.gains(3);

            deltaT_controller = t-t_last_controller_cycle;

            % for debugging
            if strcmp(current_state,"foe_turnleft") || strcmp(current_state,"foe_turnright")
                %fprintf('Turn - Angle Set: %3.2f Psi: %3.2f\n',psi_set(1),psi(1));
            else
                %fprintf('Straight - Angle Set: %3.2f Psi: %3.2f\n',psi_set(1),psi(1));
            end

            % proportional term
            e_k = psi_set-psi;
            KTerm = e_k*k;

            % integral
            if ~strcmp(state_prev,current_state)
                state_prev=current_state;
                ITerm = 0;
            end

            ITerm = ITerm + (ki * e_k * deltaT_controller);

            outMax = 0.3;
            outMin = -0.3;
            if (ITerm > outMax)
                ITerm = outMax;
            elseif (ITerm < outMin)
                ITerm = outMin;
            end

            % derivative
            decay = 0.2;
            if isnan(psi_0_filtered)
                psi_0_filtered = psi;
                e_d = 0;
            else
                psi_filtered = psi_0_filtered + (1-decay)*(psi - psi_0_filtered);
                e_d = -(psi_filtered-psi_0_filtered)/deltaT_controller;
                psi_0_filtered = psi_filtered;
            end

            DTerm = kd*e_d;

            % controller output
            DeltaL_u = KTerm + ITerm + DTerm;

            % saturation block
            saturation = 0.48;
            if DeltaL_u > saturation
                DeltaL_u = saturation;
            elseif DeltaL_u < -saturation
                DeltaL_u = -saturation;
            end
            DeltaL_u_0 = DeltaL_u;

            % Pitching (difference in length between central tether and
            % control lines
            DeltaL_power = 0;
            DeltaL_power_0 = DeltaL_power;

            % actuation
            turnrate_limiter = 1;
            [actual_DeltaL_u_L, actual_DeltaL_u_R, L_act_pos, R_act_pos] = Fun_Actuators(DeltaL_u,DeltaL_power,turnrate_limiter,deltaT);
    
            % reel-out
            reelout_length = reelout_length + reelout_speed*t_last_iter;

            % control vector
            cv = cv_init+[-actual_DeltaL_u_L;actual_DeltaL_u_R;0] + reelout_length*ones(3,1);
            
            % controller state vector, for postprocessing
            icv = writeControllerStateVector(k*e_k, ITerm, kd*e_d, delta, lambda, ...
                psi_set, psi, L_act_pos, R_act_pos, actual_DeltaL_u_L + actual_DeltaL_u_R,...
                DeltaL_u, DeltaL_g);

            % internal variables update
            icv_0 = icv;
            t_last_iter = t;
            t_last_controller_cycle = t;
    end

   
elseif (t~=t_last_iter)
    % the controller is between cycles, but the actuators are moving

    % This block is for debugging and changing initial conditions in a
    % stable manner
    % if y_last_iter<0 && xs(2)>=0 && cv_init(3) < 100.535
    %     cv_init = cv_init + 0.005*[0;0;1];% + 1*deltaT_controller;
    %     fprintf('Changind lengths at %3.2f s\n', t);
    % end

    y_last_iter = xs(2);

    DeltaL_u = DeltaL_u_0;
    DeltaL_power = DeltaL_power_0;
    turnrate_limiter = 1;
    [actual_DeltaL_u_L, actual_DeltaL_u_R, L_act_pos, R_act_pos] = Fun_Actuators(DeltaL_u,DeltaL_power,turnrate_limiter,deltaT);
    extraT = 0;

    reelout_length = reelout_length + reelout_speed*t_last_iter;

    cv = cv_init+[-actual_DeltaL_u_L;actual_DeltaL_u_R;extraT]+reelout_length*ones(3,1);


    DeltaL_g = (PD.Anchor.Points(1,2) + (-PD.Anchor.Points(2,2)))*sind(icv_0(4))*cosd(icv_0(5));

    icv = writeControllerStateVector(icv_0(1), icv_0(2), icv_0(3), icv_0(4), icv_0(5), ...
        icv_0(6), icv_0(7), L_act_pos, R_act_pos, actual_DeltaL_u_L + actual_DeltaL_u_R,...
        DeltaL_u, DeltaL_g);

    t_last_iter = t;

else
    % If the function is called again for the same timestep, just return
    % previously calculated outputs
    cv = cv_0;
    icv = icv_0;
end
end

function quadrant = angleQuadrant(angle_deg)
    angleWrapped = manual_wrapTo180(angle_deg);
    if angleWrapped > 0 && angleWrapped < 90
        quadrant = 1;
    elseif angleWrapped >= 90 && angleWrapped < 180
        quadrant = 2;
    elseif angleWrapped > -180 && angleWrapped <= -90
        quadrant = 3;
    elseif angleWrapped > -90 && angleWrapped < 0
        quadrant = 4;
    else
        quadrant = 0; % avoid edge cases for counting turns
    end
end

function icv = writeControllerStateVector(...
                                    propotional_term, ...
                                    integral_term, ...
                                    derivative_term, ...
                                    delta, ...
                                    lambda, ...
                                    psi_set, ...
                                    psi, ...
                                    L_act_pos, ...
                                    R_act_pos, ...
                                    DeltaL_u_current, ...
                                    DeltaL_u_demand, ...
                                    DeltaL_g)

    icv = zeros(12,1);
    
    icv(1) = propotional_term;
    icv(2) = integral_term;
    icv(3) = derivative_term;
    icv(4) = delta;
    icv(5) = lambda;
    icv(6) = psi_set;
    icv(7) = psi;
    icv(8) = L_act_pos;
    icv(9) = R_act_pos;
    icv(10) = DeltaL_u_current;
    icv(11) = DeltaL_u_demand;
    icv(12) = DeltaL_g;
end

function y = manual_wrapTo180(x)
    y = mod(x + 180, 360) - 180;
end

function y = manual_wrapTo360(x)
    y = mod(x, 360);
    y(y < 0) = y(y < 0) + 360;
end