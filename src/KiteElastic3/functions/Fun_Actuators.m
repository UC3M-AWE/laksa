function [actual_DeltaL_u_L, actual_DeltaL_u_R, L_act_pos, R_act_pos] = Fun_Actuators(DeltaL_u,DeltaL_power,speed_limiter,deltaT)

%-----------------------------------------------------------------------------
% Project   : LAKSA - KiteElastic3                                           %
% Authors   : F. delosRíos-Navarrete, G. Sánchez-Arriaga,                    %
% Language  : Matlab                                                         %
% Synopsis  : Simulate actuator movement                                     %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------

persistent L_Pos0 R_Pos0 L_Vel0 R_Vel0

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Physical properties %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = 0.11;
b = 0.118;
half_range_max = 157750 * (1/4096) * (1/28) * (0.15);
range_max = half_range_max*2;
Vel_max = 2724 * (1/60) * (1/28) * (0.15) * speed_limiter;
Acel_max = 20000 * (1/60) * (1/28) * (0.15);

if isempty(L_Pos0)
    L_Pos0 = half_range_max;
    R_Pos0 = half_range_max;
    L_Vel0 = 0;
    R_Vel0 = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Actuator controller %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% we get x0 for the requested delta between the central line and the others
DeltaElle = elle_from_x(half_range_max,a,b) + DeltaL_power - elle_from_x(0,a,b);
x_0 = x_from_DeltaElle(DeltaElle,a,b);

% verify it is not beyond limits
if (x_0 > range_max) || (x_0 < 0)
    error('Power line delta beyond actuator limits\n')
end

% now determine the position needed to achieve DeltaL_u
DeltaElleL = elle_from_x(x_0,a,b) + DeltaL_u/2 - elle_from_x(0,a,b);
DeltaElleR = elle_from_x(x_0,a,b) - DeltaL_u/2 - elle_from_x(0,a,b);

x_L = x_from_DeltaElle(DeltaElleL,a,b);
x_R = x_from_DeltaElle(DeltaElleR,a,b);

% check if DeltaL_u is within limits, otherwise saturate
[actual_DeltaL_u, x_L, x_R] = limitAndSaturate(x_L,x_R,x_0,DeltaL_u,range_max,a,b);
if (DeltaL_u ~= actual_DeltaL_u)
    warning('DeltaL beyond actuator capabilities. Requested %3.3f, limited at %3.3f\n',DeltaL_u,actual_DeltaL_u)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Actuator simulation %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

[pos_limited_L] = Fun_Actuator_Limits(x_L, L_Pos0, L_Vel0, range_max, Vel_max, Acel_max, deltaT);
[pos_limited_R] = Fun_Actuator_Limits(x_R, R_Pos0, R_Vel0, range_max, Vel_max, Acel_max, deltaT);

% calculate output
actual_DeltaL_u_L = (DeltaElle_from_x(pos_limited_L,a,b) - elle_from_x(x_0,a,b) + elle_from_x(0,a,b));
actual_DeltaL_u_R = -(DeltaElle_from_x(pos_limited_R,a,b) - elle_from_x(x_0,a,b) + elle_from_x(0,a,b));

L_Vel0 = (pos_limited_L - L_Pos0)/deltaT;
R_Vel0 = (pos_limited_R - R_Pos0)/deltaT;
L_Pos0 = pos_limited_L;
R_Pos0 = pos_limited_R;
L_act_pos = L_Pos0;
R_act_pos = R_Pos0;

end

%%%%%%%%%%%%%%%%%
%%% Functions %%%
%%%%%%%%%%%%%%%%%

function elle = elle_from_x(x,a,b)
    elle = sqrt(x^2+a^2) + sqrt(x^2+b^2);
end

function DeltaElle = DeltaElle_from_x(x,a,b)
    DeltaElle = sqrt(x^2+a^2) + sqrt(x^2+b^2) - a - b;
end

function x = x_from_DeltaElle(delta_elle,a,b)
    if (delta_elle*(2*a+delta_elle)*(2*b+delta_elle)*(2*a+2*b+delta_elle)) < 0
        x = -1000000;
    else
        x = sqrt(delta_elle*(2*a+delta_elle)*(2*b+delta_elle)*(2*a+2*b+delta_elle))/(2*(delta_elle+a+b));
    end
end

function [actual_DeltaL_u, x_L_sat, x_R_sat] = limitAndSaturate(x_L,x_R,x_0,DeltaL_u,range_max,a,b)
    % verify it is not beyond limits, saturate otherwise
    beyond_limits = false;
    if x_L > range_max
        x_L_sat = range_max;
        beyond_limits = true;
    elseif (x_L < 0) || ~isreal(x_L)
        x_L_sat = 0;
        beyond_limits = true;
    else
        x_L_sat = x_L;
    end
    
    if x_R > range_max
        x_R_sat = range_max;
        beyond_limits = true;
    elseif (x_R < 0) || ~isreal(x_R)
        x_R_sat = 0;
        beyond_limits = true;
    else
        x_R_sat = x_R;
    end
    
    % equalize control lines if needed
    if beyond_limits
        DeltaElle_neutral = DeltaElle_from_x(x_0,a,b);
        DeltaElle_L = DeltaElle_from_x(x_L_sat,a,b);
        DeltaElle_R = DeltaElle_from_x(x_R_sat,a,b);

        if abs(DeltaElle_neutral-DeltaElle_L) < abs(DeltaElle_neutral-DeltaElle_R)
            actual_DeltaL_u = 2*(DeltaElle_L - elle_from_x(x_0,a,b) + elle_from_x(0,a,b));
        else
            actual_DeltaL_u = -2*(DeltaElle_R - elle_from_x(x_0,a,b) + elle_from_x(0,a,b));
        end
    
        DeltaElleL = elle_from_x(x_0,a,b) + actual_DeltaL_u/2 - elle_from_x(0,a,b);
        DeltaElleR = elle_from_x(x_0,a,b) - actual_DeltaL_u/2 - elle_from_x(0,a,b);
    
        x_L_sat = x_from_DeltaElle(DeltaElleL,a,b);
        x_R_sat = x_from_DeltaElle(DeltaElleR,a,b);
    else
        actual_DeltaL_u = DeltaL_u;
        x_L_sat = x_L;
        x_R_sat = x_R;
    end
end

function [pos_limited] = Fun_Actuator_Limits(pos_demand, pos_prev, v_prev, range_max, Vel_max, Acel_max, deltaT)
    Vel = (pos_demand - pos_prev)/deltaT;
    Acc = (Vel - v_prev)/deltaT;
    
    acc_flag = false;
    if Acc > Acel_max
        Vel = Acel_max*deltaT + v_prev;
        acc_flag = true;
    elseif Acc < -Acel_max
        Vel = -Acel_max*deltaT + v_prev;
        acc_flag = true;
    end
    
    if Vel > Vel_max
        Vel = Vel_max;
        pos_limited = Vel*deltaT + pos_prev;
    elseif Vel < -Vel_max
        Vel = -Vel_max;
        pos_limited = Vel*deltaT + pos_prev;
    elseif acc_flag
        pos_limited = Vel*deltaT + pos_prev;
    else
        pos_limited = pos_demand;
    end
    
    pos_max = range_max;
    pos_min = 0;

    t_stop = abs(Vel/Acel_max);
    if Vel > 0
        pos_stop = pos_prev + t_stop*Vel - 0.5*Acel_max*t_stop^2;
        if pos_stop(1)>pos_demand(1)+0.001 || pos_stop(1)>pos_max(1)+0.001 % codegen size specified - KY
            Vel = -Acel_max*deltaT + v_prev;
            pos_limited = Vel*deltaT + pos_prev;
        end
    else
        pos_stop = pos_prev + t_stop*Vel + 0.5*Acel_max*t_stop^2;
        if pos_stop(1)<pos_demand(1)-0.001 || pos_stop(1)<pos_min(1)-0.001 % codegen size specified - KY
            Vel = Acel_max*deltaT + v_prev;
            pos_limited = Vel*deltaT + pos_prev;
        end
    end
    pos_limited = pos_limited(1); % codegen size specified - KY
    if pos_limited > pos_max
        pos_limited = pos_max;
    elseif pos_limited < pos_min
        pos_limited = pos_min;
    end
end