%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : F. delosRíos-Navarrete, G. Sánchez-Arriaga,                    %
% Language  : Matlab                                                         %
% Synopsis  : Update frame                                                   %
% Copyright:  Universidad Carlos III de Madrid, 2025. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %%
% Inputs:  handles         -> animation graphics handles                   %%
%          t               -> time                                         %%
%          u               -> state vector                                 %%
%          RBE             -> Body-Earth rotation matrix                   %%
%          PD              -> Physical Parameters                          %%
%                                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Plot_updateFrame(handles, t, u, RBE, PD)
    % Scales for easier visalization
    bScale = 2;
    xScale = 2;

    % Compute geometry:
    b  = PD.Kite.b*bScale;
    c = PD.Kite.c*xScale;
    h  = PD.Kite.h;
    hg = PD.Kite.hg;
    rk = u(1:3)'; % Kite CG position expressed in Earth frame

    % Adjust inset axis:
    axis(handles.ax2,[rk(1)-1.1*b  rk(1)+1.1*b  -rk(2)-1.5*b -rk(2)+1.5*b  -rk(3)-1.1*b  -rk(3)+1.1*b]);
    
    % Build the kite corner points in body coords
    rg = [rk(1); rk(2);rk(3)];
    REB = RBE';
    PBE = RBE*(-rg);
    TBE = [[RBE,PBE];[0,0,0,1]];
    PEB = rg;
    TEB = [[REB,PEB];[0,0,0,1]];
    cg = 3*c/4;
    et = 0.3;

    centralSpineB = [cg,-(c-cg);[0,0];[-hg,-hg]];
    leftEdgeB = [centralSpineB(:,1)+[-c-et;-b/2;h],centralSpineB(:,1)];
    rightEdgeB = [centralSpineB(:,1)+[-c-et;b/2;h],centralSpineB(:,1)];
    backLeftB = [centralSpineB(:,2),leftEdgeB(:,1)];
    backRightB = [centralSpineB(:,2),rightEdgeB(:,1)];
    
    % Transform into Earth frame
    toE = @(X) (TEB*[X; ones(1,size(X,2))]);
    centralSpineE  = toE(centralSpineB);
    leftEdgeE   = toE(leftEdgeB);
    rightEdgeE   = toE(rightEdgeB);
    backLeftE  = toE(backLeftB);
    backRightE  = toE(backRightB);
    
    % Flip Y/Z sign for plotting convention
    centralSpineE(2,:) = -centralSpineE(2,:);
    centralSpineE(3,:) = -centralSpineE(3,:);
    leftEdgeE(2,:) = -leftEdgeE(2,:);
    leftEdgeE(3,:) = -leftEdgeE(3,:);
    rightEdgeE(2,:) = -rightEdgeE(2,:);
    rightEdgeE(3,:) = -rightEdgeE(3,:);
    backLeftE(2,:) = -backLeftE(2,:);
    backLeftE(3,:) = -backLeftE(3,:);
    backRightE(2,:) = -backRightE(2,:);
    backRightE(3,:) = -backRightE(3,:);
    
    % 1) Update body‐outline lines
    verts = {centralSpineE, leftEdgeE, rightEdgeE, backLeftE, backRightE};
    for kLine = 1:length(verts)
        set(handles.lines1.body(kLine), ...
            'XData', verts{kLine}(1,:), ...
            'YData', verts{kLine}(2,:), ...
            'ZData', verts{kLine}(3,:));
        set(handles.lines2.body(kLine), ...
            'XData', verts{kLine}(1,:), ...
            'YData', verts{kLine}(2,:), ...
            'ZData', verts{kLine}(3,:));
    end
    
    % 2) Update the two triangular faces
    faceVerts1 = [centralSpineE(1:3,1)'; leftEdgeE(1:3,1)'; centralSpineE(1:3,2)'];
    faceVerts2 = [centralSpineE(1:3,1)'; rightEdgeE(1:3,1)'; centralSpineE(1:3,2)'];
    set(handles.faces1(1),'Vertices',faceVerts1);
    set(handles.faces1(2),'Vertices',faceVerts2);
    set(handles.faces2(1),'Vertices',faceVerts1);
    set(handles.faces2(2),'Vertices',faceVerts2);
    
    % 3) Update center dot
    set(handles.points1.center, ...
        'XData', rk(1), 'YData', -rk(2), 'ZData', -rk(3));
    set(handles.points2.center, ...
        'XData', rk(1), 'YData', -rk(2), 'ZData', -rk(3));
    
    % 4) Local horizontal  (in Earth frame)
    origin = rk;
    axesH = [1.1*b/2 0 0; 0 1.1*b/2 0; 0 0 1.1*b/2]';
    for ia = 1:3
        P1 = origin;
        P2 = origin + axesH(:,ia);
        set(handles.lines1.horiz(ia), ...
            'XData',[P1(1) P2(1)], ...
            'YData',[-P1(2) -P2(2)], ...
            'ZData',[-P1(3) -P2(3)]);
        set(handles.lines2.horiz(ia), ...
            'XData',[P1(1) P2(1)], ...
            'YData',[-P1(2) -P2(2)], ...
            'ZData',[-P1(3) -P2(3)]);
    end
    
    % 5) Body axes
    rf1 = rk + RBE'*[1.1*b/2 0 0]';
    rf2 = rk + RBE'*[0 1.1*b/2 0]';
    rf3 = rk + RBE'*[0 0 1.1*b/2]';
    rfs = {rf1, rf2, rf3};
    for ia=1:3
        P2 = rfs{ia};
        set(handles.lines1.bodyAx(ia), ...
            'XData',[rk(1) P2(1)], ...
            'YData',[-rk(2) -P2(2)], ...
            'ZData',[-rk(3) -P2(3)]);
        set(handles.lines2.bodyAx(ia), ...
            'XData',[rk(1) P2(1)], ...
            'YData',[-rk(2) -P2(2)], ...
            'ZData',[-rk(3) -P2(3)]);
    end
    
    % 6) Tethers + bridles
    EarthAnchors = PD.Anchor.Points';
    Connectivity = PD.Bridle.Connectivity;
    nSegments   = PD.Tether.N;
    nTethers = size(Connectivity,1);
    Bridles = PD.Bridle.Anchors;

    rm_f = 14;
    rm_e = 13 + (length(u)-13)/2;
    rm = u(rm_f:rm_e);
    r = reshape(rm,3*nSegments,[]);
    
    prev_j = 0;
    for i=1:nTethers
        tetherVector = [r(:,i) ; EarthAnchors(:,i)];
        X = tetherVector(1:3:end);
        Y = -tetherVector(2:3:end);
        Z = -tetherVector(3:3:end);
        set(handles.tethers1(i), 'XData',X,'YData',Y,'ZData',Z);
        set(handles.tethers2(i), 'XData',X,'YData',Y,'ZData',Z);
        
        % bridles
        tetherVector = [r(:,i);EarthAnchors(:,i)];
        bridlePointsB = zeros(3,length(Connectivity{i}));
        bridlePointsE = zeros(4,length(Connectivity{i}));

        for j = 1:1:length(Connectivity{i})
            bridlePointsB(:,j) = Bridles(Connectivity{i}(j),:)';
            bridlePointsE(:,j) = (TEB*[bridlePointsB(:,j);1]);

            set(handles.bridles1(j + prev_j), ...
            'XData',[tetherVector(1) bridlePointsE(1,j)], ...
            'YData',[-tetherVector(2) -bridlePointsE(2,j)], ...
            'ZData',[-tetherVector(3) -bridlePointsE(3,j)]);

            set(handles.bridles2(j + prev_j), ...
            'XData',[tetherVector(1) bridlePointsE(1,j)], ...
            'YData',[-tetherVector(2) -bridlePointsE(2,j)], ...
            'ZData',[-tetherVector(3) -bridlePointsE(3,j)]);
        end
        prev_j = j + prev_j;
    end
    
    % 7) Update title
    set(handles.title,'String',sprintf('t = %.2f s',t));
end
