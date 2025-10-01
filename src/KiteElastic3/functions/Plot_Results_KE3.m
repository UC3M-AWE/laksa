function Plot_Results_KE3(T, RBE, rk, vk, ak, euler, omega, omega_p, Lambda, FT, MT, FtetherUp, FtetherDown, FA, MA, W, alfa, beta, rm, vm, am, cv, icv, T_full, icv_full, Flag_Plot)

%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, I. Castro-Fernández,                   % 
%             A. Pastor-Rodriguez, G. Sánchez-Arriaga                        %
% Language  : Matlab                                                         %
% Synopsis  : Plot the results                                               %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %%
% Inputs:  T                 -> Time                                %%
%          Kinematics quantities of the kite                        %%
%                      RBE   -> Body-Earth rotation matrix          %%
%                      rk    -> Position (Earth), velocity (body)   %%
%                      vk       and acceleration (Earth)  vectors   %%
%                      ak       of the center of mass of the kite   %% 
%                      euler -> Euler angles (psi, theta, phi)      %%
%                      omega -> Kite angular velocity (body)        %%
%                      omega_p  and acceleration (Earth)            %%
%          Kite forces and moments                                  %%
%                      Lambda -> Tether Tensions upon kite          %%
%                                tether directions                  %%
%                      FT     -> Body components of the Forces      %%
%                               exerted by the tethers              %%
%                      MT     ->  Body components of the Torques    %%
%                               exerted by the tethers              %%
%                     FtetherDown -> Tether tension magnitude at    %%
%                                    the kite end                   %% 
%                     FtetherUp   -> Tether tension magnitude at    %%
%                                    the ground end                 %% 
%                      FA   ->  Body components of the Aerodynamic  %% 
%                      MA       force and torque                    %% 
%                      W    ->  Earth components of the kite weight %%
%           Others                                                  %%
%                      alfa -> Angle of attack                      %%  
%                      beta -> Sideslip angle                       %%
%                      LP   -> Length of the A plus-tether          %%
%                      LM   -> Length of the A minus-tether         %%
%         Masses kinematics                                         %%
%                      rm   -> Earth components of masses positions %%
%                      vm   -> Earth components of masses velocities%%
%                      am   -> Earth components of masses acc.      %%
%                      cv   -> control vector                       %%
%                      icv  -> Internal controller state vector     %%
%                      T_full -> Non-decimated time                 %%
%                      icv_full -> Non-decimated icv                %%
%                                                                   %%
%         Flag_Plot -> vector with 0/1 to set plotting options      %%
%                                                                   %%
%                      Flag_Plot(1) = 1 Plot Position               %%
%                      Flag_Plot(2) = 1 Plot velocity               %%
%                      Flag_Plot(3) = 1 Plot Euler Angles           %%
%                      Flag_Plot(4) = l Plot alfa and beta          %%
%                      Flag_Plot(5) = 1 Plot Tether tensions        %%
%                      Flag_Plot(6) = 1 Plot Tether Lengths         %%
%                      Flag_Plot(7) = 1 Plot Aero Forces            %%
%                      Flag_Plot(8) = 1 Plot Aero Torques           %%
%                      Flag_Plot(8) = 1 Plot Controller Metrics     %%
% Outputs: Figures with the results                                 %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Unit_Time       = '[s]$';
Unit_Length     = '[m]$';
Unit_Velocity   = '[m/s]$';
Unit_deg        = '[^\circ]$';
Unit_deg_vel    = '[^\circ / s]$';
Unit_Force      = '[N]$';
Unit_Torque     = '[N m]$';


if Flag_Plot(1) == 1 % Plot Position               %%
    
    figure(101)
    subplot(3,1,1)
    hold on
    plot(T,rk(1,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$X\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,rk(2,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$Y\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,3)
    hold on
    plot(T,-rk(3,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$H\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
    figure(102)
    subplot(2,1,1)
    hold on
    plot(rk(2,:),-rk(3,:))
    plot(rk(2,1),-rk(3,1),'go')
    plot(rk(2,end),-rk(3,end),'+r')
    xlabel(['$Y\ ' Unit_Length],'fontsize',12,'interpreter','latex' )
    ylabel(['$H\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
    figure(102)
    subplot(2,1,2)
    hold on
    plot(rk(1,:),-rk(3,:))
    plot(rk(1,1),-rk(3,1),'go')
    plot(rk(1,end),-rk(3,end),'+r')
    xlabel(['$X\ ' Unit_Length],'fontsize',12,'interpreter','latex' )
    ylabel(['$H\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
end
   
if Flag_Plot(2) == 1 %Plot velocity               %%
    figure(103)
    subplot(3,1,1)
    hold on
    plot(T,vk(1,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$V_x\ ' Unit_Velocity],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,vk(2,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$V_y\ ' Unit_Velocity],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,3)
    hold on
    plot(T, vk(3,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$V_z\ ' Unit_Velocity],'fontsize',12,'interpreter','latex')


    figure(104)
    hold on
    plot(T,vecnorm(vk,2,1))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$V\ ' Unit_Velocity],'fontsize',12,'interpreter','latex')
    grid on

end

if  Flag_Plot(3) == 1 %Plot Euler Angles           %%
    
    figure(105)
    hold on
    subplot(3,1,1)
    hold on
    plot(T,euler(1,:)*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\psi\ ' Unit_deg],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,euler(2,:)*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\theta\ ' Unit_deg],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,3)
    hold on
    plot(T, euler(3,:)*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\phi\ ' Unit_deg],'fontsize',12,'interpreter','latex')
end

if Flag_Plot(4) == 1 %Plot alfa and beta          %%
    figure(106)
    subplot(2,1,1)
    hold on
    plot(T,alfa*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\alpha\ ' Unit_deg],'fontsize',12,'interpreter','latex')
    
    subplot(2,1,2)
    hold on
    plot(T,beta*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\beta\ ' Unit_deg],'fontsize',12,'interpreter','latex')

    
    for i=2:length(alfa)
        dAlpha(i-1) = (alfa(i)-alfa(i-1))/(T(i)-T(i-1));
        dBeta(i-1) = (beta(i)-beta(i-1))/(T(i)-T(i-1));
    end

    figure(107)
    subplot(2,1,1)
    hold on
    plot(T(2:end),dAlpha)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$d\alpha / dt\ ' Unit_deg_vel],'fontsize',12,'interpreter','latex')

    subplot(2,1,2)
    hold on
    plot(T(2:end),dBeta*180/pi)
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$d\beta / dt\ ' Unit_deg_vel],'fontsize',12,'interpreter','latex')
end

if Flag_Plot(5) == 1 %% Plot Tether tensions        %%
    tension_smooth = smooth(FtetherUp(:,3),0.03,'lowess');
    [Maxima,MaxIdx] = findpeaks(tension_smooth);
    DataInv = 1.01*max(tension_smooth) - tension_smooth;
    [Minima,MinIdx] = findpeaks(DataInv);
    
    figure(108)
    subplot(2,1,2)
    hold on
    grid on
    %ylim([20 40])
    plot(-rk(2,:),-rk(3,:))
    plot(-rk(2,MaxIdx),-rk(3,MaxIdx),'^','MarkerEdgeColor','red','MarkerFaceColor','red')
    plot(-rk(2,MinIdx),-rk(3,MinIdx),'v','MarkerEdgeColor','magenta','MarkerFaceColor','magenta')
    % plot(rk(2,end),-rk(3,end),'+r')
    xlabel('Y [m]','fontsize',14,'interpreter','latex' )
    ylabel('H [m]','fontsize',14,'interpreter','latex')
    
    subplot(2,1,1)
    hold on
    hold on
    lt = plot(T,FtetherUp(:,1),'g');
    lt.Color = "#EDB120";
    lt.LineWidth = 1.2;
    rt = plot(T,FtetherUp(:,2),'');
    rt.Color = "#77AC30";
    rt.LineWidth = 1.2;
    ct = plot(T,FtetherUp(:,3));
    ct.Color = "#0072BD";
    ct.LineWidth = 1.2;
    plot(T(MaxIdx),FtetherUp(MaxIdx,3),'^','MarkerEdgeColor','red','MarkerFaceColor','red')
    plot(T(MinIdx),FtetherUp(MinIdx,3),'v','MarkerEdgeColor','magenta','MarkerFaceColor','magenta')
    set(gca,'xtick',0:1:floor(T(end)))
    
    grid on
    xlim([0 T(end)])
    %ylim([0 400])
    xlabel(['$T\ ' Unit_Time],'fontsize',14,'interpreter','latex' )
    ylabel('Tether tension $[N]$','fontsize',14,'interpreter','latex')



    figure(109)
    hold on
    plot(T,sum(FtetherUp,2))
    xlabel(['$T\ ' Unit_Time],'fontsize',14,'interpreter','latex' )
    ylabel('Total tether tension $[N]$','fontsize',14,'interpreter','latex')
    grid on

end


if Flag_Plot(6) == 1 % Plot Tether Lengths         %%
    figure(110)
    subplot(3,1,1)
    hold on
    plot(T,cv(1,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$L_L\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,cv(2,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$L_R\ ' Unit_Length],'fontsize',12,'interpreter','latex')

    subplot(3,1,3)
    hold on
    plot(T,cv(3,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$L_C\ ' Unit_Length],'fontsize',12,'interpreter','latex')
end


if Flag_Plot(7) == 1 % Plot aerodynamic forces         %%
    figure(111)
    subplot(3,1,1)
    hold on
    plot(T,FA(1,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$F_{A,x}\ ' Unit_Force],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,FA(2,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$F_{A,y}\ ' Unit_Force],'fontsize',12,'interpreter','latex')

    subplot(3,1,3)
    hold on
    plot(T,FA(3,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$F_{A,z}\ ' Unit_Force],'fontsize',12,'interpreter','latex')


end

if Flag_Plot(8) == 1 % Plot aerodynamic torques         %%
    figure(112)
    subplot(3,1,1)
    hold on
    plot(T,MA(1,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$M_{A,x}\ ' Unit_Torque],'fontsize',12,'interpreter','latex')
    
    subplot(3,1,2)
    hold on
    plot(T,MA(2,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$M_{A,y}\ ' Unit_Torque],'fontsize',12,'interpreter','latex')

    subplot(3,1,3)
    hold on
    plot(T,MA(3,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$M_{A,z}\ ' Unit_Torque],'fontsize',12,'interpreter','latex')
end

if Flag_Plot(9) == 1 % Plot controller metrics         %%
    figure(113)
    hold on
    plot(T,icv(6,:))
    plot(T,icv(7,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['Heading angle $' Unit_deg],'fontsize',12,'interpreter','latex')
    legend({'$\psi_{set}$','$\psi$'},'fontsize',10,'interpreter','latex')
    grid on
    
    
    figure(114)
    subplot(2,1,1)
    hold on
    plot(T,icv(10,:))
    plot(T,icv(11,:))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel(['$\Delta L\ ' Unit_Length],'fontsize',12,'interpreter','latex')
    legend({'$\Delta L_i$','$\Delta L_{set}$'},'fontsize',14,'interpreter','latex')
    grid on
    subplot(2,1,2)
    hold on
    grid on
    plot(T,icv(10,:))
    xlim([0 T(end)])
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    ylabel('$\Delta L\ [m]$','fontsize',10,'interpreter','latex')

    figure(115)
    hold on
    plot(icv(4,:),icv(5,:))
    plot(-25,35,'.')
    plot(25,35,'.')
    plot(-50,27,'.')
    plot(50,27,'.')
    viscircles([-25,35],15)
    viscircles([25,35],10)
    set(gca,'xdir','reverse')
    xlabel(['$\delta\ ' Unit_deg],'fontsize',14,'interpreter','latex' )
    ylabel(['$\lambda\ ' Unit_deg],'fontsize',14,'interpreter','latex')
    grid on
    axis equal
    
    figure(116)
    hold on
    plot(T,icv(10,:))
    plot(T,icv(12,:))
    plot(T,icv(10,:)+icv(12,:))
    xlabel(['$T\ ' Unit_Time],'fontsize',14,'interpreter','latex' )
    ylabel(['Control input\ $' Unit_Length],'fontsize',14,'interpreter','latex')
    legend({'Controller','Geometric','Total'},'fontsize',10,'interpreter','latex')
    grid on
    
    figure(117)
    hold on
    plot(T_full(isfinite(icv_full(1,:))),icv_full(1,isfinite(icv_full(1,:))))
    plot(T_full(isfinite(icv_full(2,:))),icv_full(2,isfinite(icv_full(2,:))))
    plot(T_full(isfinite(icv_full(3,:))),icv_full(3,isfinite(icv_full(3,:))))
    xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
    legend({'$k_p \cdot$ err','$k_i \cdot$ err','$k_d \cdot$ err'},'fontsize',14,'interpreter','latex')
    grid on

end











end