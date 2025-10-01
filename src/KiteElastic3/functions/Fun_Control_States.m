function [next_state,controller_parameters] = Fun_Control_States(t,psi,delta,lambda,current_state)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, G. Sánchez-Arriaga,                    %
% Language  : Matlab                                                         %
% Synopsis  : Provide guidance and control parameters and state flow         %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %%
% Inputs:  t               -> time                                         %%
%          psi             -> heading angle                                %%
%          delta           -> azimuth angle                                %%
%          lambda          -> elevation angle                              %%
%          current_state   -> current state                                %%
%                                                                          %%
% Outputs: next_state            -> next state                             %%
%          controller_parameters -> controller parameters                  %%
%                                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

controller_parameters = struct;
next_state = '';

switch current_state
    case 'none'
        % Machine-state flow
        next_state = 'open-loop_zenith-test';
        % Controller
        controller_parameters.waypoint_type = 'none';
        controller_parameters.waypoint = [NaN,NaN];
        controller_parameters.gains = [NaN,NaN,NaN];

    case 'open-loop'
        % Machine-state flow
        next_state = 'open-loop';
        % Controller
        controller_parameters.waypoint_type = 'none';
        controller_parameters.waypoint = [NaN,NaN];
        controller_parameters.gains = [NaN,NaN,NaN];

    case 'open-loop_zenith-test'
        % Machine-state flow
        if (t > 3)
            next_state = 'zenith';
        end
        % Controller
        controller_parameters.waypoint_type = 'fixed_input';
        controller_parameters.waypoint = [-0.2,0.00];
        controller_parameters.gains = [NaN,NaN,NaN];

    case 'zenith'
        % Machine-state flow
        if (t > 35)
            next_state = 'descend_1';
        end
        % Controller
        controller_parameters.waypoint_type = 'attractor';
        controller_parameters.waypoint = [0,85];
        controller_parameters.gains = [-8*(1/1000),-0.2*(1/1000),-6*(1/1000)];

    case 'descend_1'
        % Machine-state flow
        if (delta > 40)
            next_state = 'descend_2';
        end
        % Controller
        controller_parameters.waypoint_type = 'attractor';
        controller_parameters.waypoint = [40,70];
        controller_parameters.gains = [-6*(1/1000),-0.5*(1/1000),-1*(1/1000)];

    case 'descend_2'
        % Machine-state flow
        if (delta < -45)
            next_state = 'foe_goleft';
        end
        % Controller
        controller_parameters.waypoint_type = 'attractor';
        controller_parameters.waypoint = [-70,50];
        controller_parameters.gains = [-6*(1/1000),0*(1/1000),-3*(1/1000)];

    case 'foe_turnright'
        % Machine-state flow
        if (psi > -5)
            next_state = 'foe_goright';
        end
        % Controller
        controller_parameters.waypoint_type = 'centre';
        controller_parameters.waypoint = [25,35];
        controller_parameters.gains = [-6*(1/1000),0*(1/1000),-1*(1/1000)];

    case 'foe_goright'
        % Machine-state flow
        if (delta < -25)
            next_state = 'foe_turnleft';
        end
        % Controller
        controller_parameters.waypoint_type = 'attractor';
        controller_parameters.waypoint = [-50,27];
        controller_parameters.gains = [-6*(1/1000),0*(1/1000),-1*(1/1000)];

    case 'foe_turnleft'
        % Machine-state flow
        if (psi < 5)
            next_state = 'foe_goleft';
        end
        % Controller
        controller_parameters.waypoint_type = 'centre';
        controller_parameters.waypoint = [-25,35];
        controller_parameters.gains = [-6*(1/1000),0*(1/1000),-1*(1/1000)];

    case 'foe_goleft'
        % Machine-state flow
        if (delta > 25)
            next_state = 'foe_turnright';
        end
        % Controller
        controller_parameters.waypoint_type = 'attractor';
        controller_parameters.waypoint = [50,27];
        controller_parameters.gains = [-6*(1/1000),0*(1/1000),-1*(1/1000)];

    otherwise
        % Machine-state flow
        next_state = current_state;
        % Controller
        controller_parameters.waypoint_type = 'none';
        controller_parameters.waypoint = [];
        controller_parameters.gains = [];

end

if strcmp(next_state,'')
    next_state = current_state;
end