%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, G. Sánchez-Arriaga,                    %
% Language  : Matlab                                                         %
% Synopsis  : Initialise kite animation elements                             %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                               %%
% Inputs: ax  -> Axis handle                                    %%
%                                                               %%
% Outputs: Animation elements handles                           %%
%                                                               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [points, lines, faces] = Plot_initKiteObjects(ax)
    axes(ax);
    
    % 1) Body outlines (5 lines)
    lines.body = gobjects(5,1);
    for i=1:5
        lines.body(i) = plot3(nan,nan,nan,'b','LineWidth',1.5);
    end
    
    % 2) Faces (2 patches)
    faces = gobjects(2,1);
    for i=1:2
        faces(i) = patch('Vertices',zeros(4,3),'Faces',[1 2 3],...
            'FaceColor','c','Parent',ax);
    end
    
    % 3) Center‐of‐mass marker
    points.center = plot3(nan,nan,nan,'o','MarkerSize',4,...
        'MarkerFaceColor','k','MarkerEdgeColor','k');
    
    % 4) Local horizontal axes (red)
    lines.horiz = gobjects(3,1);
    for i=1:3
        lines.horiz(i) = plot3(nan,nan,nan,'r','LineWidth',0.1);
    end
    
    % 5) Body axes (blue)
    lines.bodyAx = gobjects(3,1);
    for i=1:3
        lines.bodyAx(i) = plot3(nan,nan,nan,'b','LineWidth',0.1);
    end
end
