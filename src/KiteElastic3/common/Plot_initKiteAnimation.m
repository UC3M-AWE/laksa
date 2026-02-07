%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, G. Sánchez-Arriaga,                    %
% Language  : Matlab                                                         %
% Synopsis  : Initialise animation elements                                  %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                               %%
% Inputs: PD  -> Physical Parameters                            %%
%                                                               %%
% Outputs: Animation elements handles                           %%
%                                                               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handles = Plot_initKiteAnimation(PD)
    % Create a fixed‐size figure
    handles.fig = figure(23);
    set(handles.fig,'Position',[100 100 1000 600]);
    
    % Main axes
    handles.ax1 = axes('Position',[0.1 0.1 0.7 0.7]);
    view(handles.ax1,[120 20]);
    grid(handles.ax1,'on');
    axis(handles.ax1,'equal');
    axis(handles.ax1,[-1.1 0 -1 1 -1.1 0]*max(PD.Tether.L0));
    xlabel(handles.ax1,'x (m)');
    ylabel(handles.ax1,'y (m)');
    zlabel(handles.ax1,'z (m)');
    hold(handles.ax1,'on');
    set(gca, 'YDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    
    % Inset axes
    handles.ax2 = axes('Position',[0.65 0.7 0.28 0.28]);
    view(handles.ax2,[120 20]);
    grid(handles.ax2,'on');
    axis(handles.ax2,'equal');
    hold(handles.ax2,'on');
    handles.ax2.XTickLabel = '';
    handles.ax2.YTickLabel = '';
    handles.ax2.ZTickLabel = '';
    set(gca, 'YDir', 'reverse')
    set(gca, 'ZDir', 'reverse')

    % Pre‐create tether lines (main + inset)
    nTethers = size(PD.Bridle.Connectivity,1);
    handles.tethers1 = gobjects(nTethers,1);
    handles.tethers2 = gobjects(nTethers,1);
    for j=1:nTethers
        handles.tethers1(j) = plot3(handles.ax1,nan,nan,nan,'Color','#8A8281');
        handles.tethers2(j) = plot3(handles.ax2,nan,nan,nan,'Color','#8A8281');
    end

    % Pre‐create bridles
    Connectivity = PD.Bridle.Connectivity;
    prev_j = 0;
    for i=1:1:nTethers
        for j = 1:1:length(Connectivity{i})
            handles.bridles1(j + prev_j) = plot3(handles.ax1,nan,nan,nan,'Color','#8A8281');
            handles.bridles2(j + prev_j) = plot3(handles.ax2,nan,nan,nan,'Color','#8A8281');
        end
        prev_j = j + prev_j;
    end

    % Pre‐create kite graphics on each axes
    [handles.points1, handles.lines1, handles.faces1] = Plot_initKiteObjects(handles.ax1);
    [handles.points2, handles.lines2, handles.faces2] = Plot_initKiteObjects(handles.ax2);
    
    % Title handle
    handles.title = title(handles.ax1,'');
end

